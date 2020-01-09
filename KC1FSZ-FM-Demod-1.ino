/*
FM demodulation using I/Q and other DSP techniques.

Bruce MacKinnon KC1FSZ

For Teensy 4.0: https://www.pjrc.com/teensy/IMXRT1060RM_rev1.pdf

  NOTE: SIGNIFICANT PARTS OF THE TEENSY CODE HAVE BEEN ADAPTED FROM THE ORIGINAL
  TEENSY AUDIO LIBRARY BY PAUL STOFFREGEN.  HIS COPYRIGHT FOLLOWS BELOW:

 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 */
#include <Audio.h>
#include <arm_math.h>

/**
13.6.1.3.4 Audio PLL (PLL4)
The audio PLL synthesize a low jitter clock from a 24 MHz reference clock. The clock
output frequency range for this PLL is from 650 MHz to 1.3 GHz. It has a Fractional-N
synthesizer.

There are /1, /2, /4 post dividers for the Audio PLL. The output frequency can be set by
programming the fields in the CCM_ANALOG_PLL_AUDIO, and CCM_ANALOG_MISC2 register sets according 
to the following equation: 

PLL output frequency = Fref * (DIV_SELECT + NUM / DENOM)
 */
PROGMEM
void set_audioClock2(int nfact, int32_t nmult, uint32_t ndiv, bool force = false) // sets PLL4
{
  if (!force && (CCM_ANALOG_PLL_AUDIO & CCM_ANALOG_PLL_AUDIO_ENABLE)) return;

  CCM_ANALOG_PLL_AUDIO = CCM_ANALOG_PLL_AUDIO_BYPASS | CCM_ANALOG_PLL_AUDIO_ENABLE
           | CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(2) // 2: 1/4; 1: 1/2; 0: 1/1
           | CCM_ANALOG_PLL_AUDIO_DIV_SELECT(nfact);

  CCM_ANALOG_PLL_AUDIO_NUM   = nmult & CCM_ANALOG_PLL_AUDIO_NUM_MASK;
  CCM_ANALOG_PLL_AUDIO_DENOM = ndiv & CCM_ANALOG_PLL_AUDIO_DENOM_MASK;
  
  CCM_ANALOG_PLL_AUDIO &= ~CCM_ANALOG_PLL_AUDIO_POWERDOWN;//Switch on PLL
  while (!(CCM_ANALOG_PLL_AUDIO & CCM_ANALOG_PLL_AUDIO_LOCK)) {}; //Wait for pll-lock
  
  const int div_post_pll = 1; // other values: 2,4
  CCM_ANALOG_MISC2 &= ~(CCM_ANALOG_MISC2_DIV_MSB | CCM_ANALOG_MISC2_DIV_LSB);
  if(div_post_pll>1) CCM_ANALOG_MISC2 |= CCM_ANALOG_MISC2_DIV_LSB;
  if(div_post_pll>3) CCM_ANALOG_MISC2 |= CCM_ANALOG_MISC2_DIV_MSB;
  
  CCM_ANALOG_PLL_AUDIO &= ~CCM_ANALOG_PLL_AUDIO_BYPASS;//Disable Bypass
}

// This is the size of the buffer used for DMA transmit/receive operations.
// This needs to be twice the block size since the interrupt will be configured
// to fire at the half-way mark.
const unsigned int tx_buffer_size = 128;
const unsigned int rx_buffer_size = 128;

// Create the buffer from which DMA transmit/receive operations will happen.
//
// (From PJS) "DMAMEM is not required. It only serves to place your buffers lower in memory.
// The idea is typical programs will do most of their ordinary memory access to the stack, 
// located in the upper memory. If your buffers are in lower memory, odds are (maybe) less 
// of the memory controller adding a wait state if the CPU and DMA want to access the same 
// region of memory in the same clock cycle.
DMAMEM __attribute__((aligned(32))) static uint32_t i2s_tx_buffer[tx_buffer_size];

static uint32_t i2s_rx_buffer[rx_buffer_size];

// ====== DMA Stuff ====================================================================

extern "C" {
  extern uint16_t dma_channel_allocated_mask;
}

// This structure matches the layout of the DMA Transfer Control Descriptor
//
struct __attribute__((packed, aligned(4))) TCD {    
  // Source Address 
  volatile const void * volatile SADDR;
  // Signed Source Address Offset - Sign-extended offset applied to the current source 
  // address to form the next-state value as each source read is completed.
  int16_t SOFF;
  // Transfer Attributes
  uint16_t ATTR;  
  // TCD word 2's register definition depends on the status of minor loop mapping. 
  union { 
    // (DISALBLED) Number of bytes to be transferred in each service request of the channel. 
    uint32_t NBYTES; 
    // (ENABLED) SMLOE/DMLOE/Minor Byte Transfer Count
    uint32_t NBYTES_MLNO;
    uint32_t NBYTES_MLOFFNO; 
    uint32_t NBYTES_MLOFFYES; 
  };
  // Adjustment value added to the source address at the completion of the major iteration count. 
  int32_t SLAST;
  // Destination address
  volatile void * volatile DADDR;
  // Signed Source Address Offset - Sign-extended offset applied to the current destination
  // address to form the next-state value as each source read is completed.  
  int16_t DOFF;
  // Current channel linking feature/major iteration count
  union { 
    volatile uint16_t CITER;
    volatile uint16_t CITER_ELINKYES; 
    volatile uint16_t CITER_ELINKNO; 
  };
  // Destination last address adjustment or the memory address for the next transfer control 
  // descriptor to be loaded into this channel (scatter/gather).
  int32_t DLASTSGA;
  // Control and Status
  volatile uint16_t CSR;
  //  Beginning Minor Loop Link
  union { 
    volatile uint16_t BITER;
    volatile uint16_t BITER_ELINKYES; 
    volatile uint16_t BITER_ELINKNO; 
  };
};

/**
 * Gets the TCD structure for the specified channel.
 */
struct TCD* DMAChannel_getTCD(uint8_t channel) {
  return (struct TCD*)(0x400E9000 + (uint32_t)channel * 32); 
}

/**
 * Allocates the next available DMA channel and initializes it.
 */
uint8_t DMAChannel_begin() {

  // Search for available channel
  __disable_irq();
  
  uint32_t ch = 0;
  
  while (true) {
    if (!(dma_channel_allocated_mask & (1 << ch))) {
      dma_channel_allocated_mask |= (1 << ch);
      __enable_irq();
      break;
    }
    // Look for case where there are no channels available
    if (++ch >= 16) {
      __enable_irq();
      return 16;
    }
  }

  // Clock control
  // Clock Gating Register 5 - Enable DMA clock
  CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);

  // DMA control register
  // Group 1 priority, minor loop enabled, debug enabled
  DMA_CR = DMA_CR_GRP1PRI | DMA_CR_EMLM | DMA_CR_EDBG;  
  DMA_CERQ = ch;
  DMA_CERR = ch;
  DMA_CEEI = ch;
  DMA_CINT = ch;

  // Clear control structure
  struct TCD* tcd = DMAChannel_getTCD(ch);
  //uint32_t *p = (uint32_t*)tcd;
  //for (int i = 0; i < 8; i++)
  //  *p++ = 0;
  memset((void*)tcd,0,32);
    
  return ch;
}

static void DMAChannel_triggerAtHardwareEvent(uint8_t s,uint8_t channel) {
  // Get the mux setup 
  volatile uint32_t* mux = &DMAMUX_CHCFG0 + channel;
  *mux = 0;
  *mux = (s & 0x7F) | DMAMUX_CHCFG_ENBL;
}

static void DMAChannel_enable(uint8_t channel) {
  DMA_SERQ = channel;
}

static void DMAChannel_attachInterrupt(uint8_t channel,void (*isr)(void)) {
  // _VectorsRam is created by the Teensy library and it has been initialized
  // using the flash vector table. 
  //
  // void (* _VectorsRam[NVIC_NUM_INTERRUPTS+16])(void);
  //
  // This is the code that maps the RAM table:
  //
  // SCB_VTOR = (uint32_t)_VectorsRam; // use vector table in RAM
  //
  // There are 16 extra entries in the table before the normal IRQ entries
  //
  _VectorsRam[16 + IRQ_DMA_CH0 + channel] = isr;
  NVIC_ENABLE_IRQ(IRQ_DMA_CH0 + channel);
}

static void DMAChannel_clearInterrupt(uint8_t channel) {
  // The INT register provides a bit map for the 16 channels signaling the presence of an
  // interrupt request for each channel. Depending on the appropriate bit setting in the
  // transfer-control descriptors, the eDMA engine generates an interrupt on data transfer
  // completion. The outputs of this register are directly routed to the interrupt controller
  // (INTC). During the interrupt-service routine associated with any given channel, it is the
  // software’s responsibility to clear the appropriate bit, negating the interrupt request.
  // Typically, a write to the CINT register in the interrupt service routine is used for this
  // purpose.
  DMA_CINT = channel;
}

void vector_diff(float32_t* a,float32_t* b,float32_t* r,int blockSize) {
  for (int i = 0; i < blockSize; i++) {
    r[i] = a[i] - b[i];
  }
}

void vector_sum(float32_t* a,float32_t* b,float32_t* r,int blockSize) {
  for (int i = 0; i < blockSize; i++) {
    r[i] = a[i] + b[i];
  }
}

void vector_mult(float32_t* a,float32_t* b,float32_t* r,int blockSize) {
  for (int i = 0; i < blockSize; i++) {
    r[i] = a[i] * b[i];
  }
}

void vector_scale(float32_t* a,float32_t b,float32_t* r,int blockSize) {
  for (int i = 0; i < blockSize; i++) {
    r[i] = a[i] * scale;
  }
}

/**
 * Used to implement a one tap delay across a block of samples
 */
class DelayLine {
public:

  DelayLine(int blockSize) : _blockSize(blockSize), _state(0) {
  }

  void process(float32_t* input,float32_t* output) {
      // Recycle the oldest entry
      output[0] = _state;
      for (int i = 0; i < _blockSize - 1; i++) {
        output[i+1] = input[i];
      }
      // The newest entry in the input gets saved for the next block
      _state = input[_blockSize - 1];
  }

private:

  const int _blockSize;
  float32_t _state;
};

// TODO: GET RID OF THIS
AudioControlSGTL5000  sgtl5000_1;

uint8_t TX_DMA_Channel = 0;
uint8_t RX_DMA_Channel = 0;

// Data transfer area from RX to TX.
// This is organized into an 8-slot circular buffer, each slot is 
// 64 16-bit words.
// Head points to next slot to be written and is moved forward on each write.
// Tail points to next slot to be read and is moved forward on each read.  This is only 
// allowed when the head != tail.
//
const int TransferSize = 8;
volatile int16_t TransferL[TransferSize][64];
volatile int16_t TransferR[TransferSize][64];
volatile int TransferHead = 0;
volatile int TransferTail = 0;

// This is where we actually generate the transmit data.
//
void make_tx_data(uint32_t txBuffer[],unsigned int txBufferSize) {  

  for (unsigned int i = 0; i < txBufferSize && i < 64; i++) {
    // We need to shift the data up to the high side of the 32-bit word
    uint32_t s_right = (uint32_t)TransferR[TransferTail][i] << 16;
    uint32_t s_left = (uint32_t)TransferL[TransferTail][i];
    txBuffer[i] = s_left | s_right;
  }

  if (++TransferTail == TransferSize) {
    TransferTail = 0;
  }
}

volatile bool CaptureEnabled = false;
volatile bool AnalysisBlockAvailable = false;
volatile int AnalysisBlockPtr = 0;
volatile float32_t AnalysisBlockL[1024];
volatile float32_t AnalysisBlockR[1024];
volatile int AnalysisSide = 0; 

static arm_fir_instance_f32 LPF_Instance;

static DelayLine Delay_I1(64);
static DelayLine Delay_I2(64);
static DelayLine Delay_Q1(64);
static DelayLine Delay_Q2(64);

static float32_t amp = 1.0;

// This is where we actually consume the receive data.
void consume_rx_data(uint32_t rxBuffer[],unsigned int rxBufferSize) {

  // Make left and right channels from what came in via DMA
  float32_t left_data[64], right_data[64];
  for (unsigned int i = 0; i < rxBufferSize && i < 64; i++) {
    left_data[i] = (int16_t)(rxBuffer[i] & 0xffff);
    right_data[i] = (int16_t)((rxBuffer[i] & 0xffff0000) >> 16);
  }

  float32_t i1[64],i2[64],q1[64],q2[64];
  float32_t a1[64],a2[64],b1[64],b2[64];
  float32_t r[64];

  Delay_I1.process(left_data,i1);
  Delay_I2.process(i1,i2);
  
  Delay_Q1.process(right_data,q1);
  Delay_Q2.process(q1,q2);

  vector_diff(left_data,i2,a1,64);
  vector_diff(right_data,q2,a2,64);

  vector_mult(i1,a2,b1,64);
  vector_mult(q1,a1,b2,64);
  
  vector_sum(b1,b2,r,64);

  // Scale and capture the data into the feedthrough buffer.  
  vector_scale(r,amp,TransferL[TransferHead],64);

  // Rotate 
  if (++TransferHead == TransferSize) {
    TransferHead = 0;
  }
}

// Interrupt service routine from DMA controller
void tx_dma_isr_function(void) {  

  const struct TCD* tcd = DMAChannel_getTCD(TX_DMA_Channel);
  uint32_t saddr = (uint32_t)tcd->SADDR;
 
  // Clear interrupt request for channel
  DMAChannel_clearInterrupt(TX_DMA_Channel);

  // Check to see what part of the TX buffer the DMA channel is working on.  If 
  // the DMA is working on the first half of the buffer then the new data should
  // be flowed into the second half.
  int startPtr;
  if (saddr < (uint32_t)i2s_tx_buffer + sizeof(i2s_tx_buffer) / 2) {
    startPtr = tx_buffer_size / 2;
  } else {
    startPtr = 0;
  }

  // Request to generate a block of data
  make_tx_data(&(i2s_tx_buffer[startPtr]),tx_buffer_size / 2);
  // Needed to allow DMA to see data  
  arm_dcache_flush_delete(&(i2s_tx_buffer[startPtr]),sizeof(i2s_tx_buffer) / 2);
}

// Interrupt service routine from DMA controller
void rx_dma_isr_function(void) {  

  const struct TCD* tcd = DMAChannel_getTCD(RX_DMA_Channel);
  uint32_t daddr = (uint32_t)tcd->DADDR;

  // Clear interrupt request for channel
  DMAChannel_clearInterrupt(RX_DMA_Channel);

  // Check to see what part of the RX buffer the DMA channel is working on.  If 
  // the DMA is working on the first half of the buffer then the new data should
  // be flowed into the second half.
  int startPtr;
  if (daddr < (uint32_t)i2s_rx_buffer + sizeof(i2s_rx_buffer) / 2) {
    startPtr = rx_buffer_size / 2;
  } else {
    startPtr = 0;
  }

  // Request to consume a block of data
  consume_rx_data(&(i2s_rx_buffer[startPtr]),rx_buffer_size / 2);
}

// ===== I2S Setup ================================================================

static void AudioOutputI2S_config_i2s() {
  
  // Enable clock for SAI1 module
  CCM_CCGR5 |= CCM_CCGR5_SAI1(CCM_CCGR_ON);

  // If either transmitter or receiver is enabled, do nothing (prevents multiple setups)
  if (I2S1_TCSR & I2S_TCSR_TE) return;
  if (I2S1_RCSR & I2S_RCSR_RE) return;

  int fs = 44100.0;
  // PLL between 27*24 = 648 MHz and 54*24=1296 MHz
  // SAI prescaler 4 => (n1*n2) = multiple of 4
  int n1 = 4;
  int n2 = 1 + (24000000 * 27) / (fs * 256 * n1);
  double C = ((double)fs * 256 * n1 * n2) / 24000000;
  int c0 = C;
  int c2 = 10000;
  int c1 = C * c2 - (c0 * c2);

  set_audioClock2(c0,c1,c2);

  // clear SAI1_CLK register locations
  // &0x03 
  // (0,1,2): PLL3PFD0, PLL5, PLL4
  CCM_CSCMR1 = (CCM_CSCMR1 & ~(CCM_CSCMR1_SAI1_CLK_SEL_MASK))
       | CCM_CSCMR1_SAI1_CLK_SEL(2); 
  CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI1_CLK_PRED_MASK | CCM_CS1CDR_SAI1_CLK_PODF_MASK))
       | CCM_CS1CDR_SAI1_CLK_PRED(n1-1) // &0x07
       | CCM_CS1CDR_SAI1_CLK_PODF(n2-1); // &0x3f

  // Select MCLK
  IOMUXC_GPR_GPR1 = (IOMUXC_GPR_GPR1
    & ~(IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL_MASK))
    | (IOMUXC_GPR_GPR1_SAI1_MCLK_DIR | IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL(0));

  CORE_PIN23_CONFIG = 3;  // 1:MCLK
  CORE_PIN21_CONFIG = 3;  // 1:RX_BCLK
  CORE_PIN20_CONFIG = 3;  // 1:RX_SYNC

  int rsync = 0;
  int tsync = 1;

  // CONFIGURE TRANSMIT
  // No mask - we are sending both channels of audio
  I2S1_TMR = 0;
  // Set the high water mark for the FIFO.  This determines when the interupt
  // needs to fire.
  I2S1_TCR1 = I2S_TCR1_RFW(1);
  I2S1_TCR2 = I2S_TCR2_SYNC(tsync) | I2S_TCR2_BCP 
        | (I2S_TCR2_BCD | I2S_TCR2_DIV((1)) | I2S_TCR2_MSEL(1));
  I2S1_TCR3 = I2S_TCR3_TCE;
  I2S1_TCR4 = I2S_TCR4_FRSZ((2-1)) | I2S_TCR4_SYWD((32-1)) | I2S_TCR4_MF
        | I2S_TCR4_FSD | I2S_TCR4_FSE | I2S_TCR4_FSP;
  I2S1_TCR5 = I2S_TCR5_WNW((32-1)) | I2S_TCR5_W0W((32-1)) | I2S_TCR5_FBT((32-1));

  // CONFIGURE RECEIVE
  I2S1_RMR = 0;
  I2S1_RCR1 = I2S_RCR1_RFW(1);
  I2S1_RCR2 = I2S_RCR2_SYNC(rsync) | I2S_RCR2_BCP  // sync=0; rx is async;
        | (I2S_RCR2_BCD | I2S_RCR2_DIV((1)) | I2S_RCR2_MSEL(1));
  I2S1_RCR3 = I2S_RCR3_RCE;
  I2S1_RCR4 = I2S_RCR4_FRSZ((2-1)) | I2S_RCR4_SYWD((32-1)) | I2S_RCR4_MF
        | I2S_RCR4_FSE | I2S_RCR4_FSP | I2S_RCR4_FSD;
  I2S1_RCR5 = I2S_RCR5_WNW((32-1)) | I2S_RCR5_W0W((32-1)) | I2S_RCR5_FBT((32-1));
}

static void AudioOutputI2S_begin() {

  // Setup DMA channel, assume this gets called second
  TX_DMA_Channel = DMAChannel_begin();

  AudioOutputI2S_config_i2s();
  
  CORE_PIN7_CONFIG = 3; // 1:TX_DATA0
  
  struct TCD* tcd = DMAChannel_getTCD(TX_DMA_Channel);
  // Configure the source source buffer
  tcd->SADDR = i2s_tx_buffer;
  // Amount that the source address is adjusted after each read.  
  // Note that we are handling 16-bit data so this value is 2 bytes.
  tcd->SOFF = 2;
  // Source and destination modulo disabled.  Source and destination transfer
  // sizes are 16 bits.
  tcd->ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
  // Minor loop mapping is disabled.  Minor byte transfer count.
  tcd->NBYTES_MLNO = 2;
  // Adjustment value added to the source address at the completion of the major iteration count. 
  // Moving things back to the initial value.
  tcd->SLAST = -sizeof(i2s_tx_buffer);
  // Sign-extended offset applied to the current destination address to form the next-state 
  // value as each destination write is completed. No movement needed here.
  tcd->DOFF = 0;
  // Channel linking is disabled, current major iteration count. This is half because
  // each tranfer moves two byes.
  tcd->CITER_ELINKNO = sizeof(i2s_tx_buffer) / 2;
  // Scatter/gather not used
  tcd->DLASTSGA = 0;
  // Channel linking is disabled, begin major iteration count
  tcd->BITER_ELINKNO = sizeof(i2s_tx_buffer) / 2;
  // Generate interrupt when major counter is half done.  Specifically, the comparison
  // performed by the eDMA engine is (CITER == (BITER >> 1)). 
  // Generate interrupt when major counter completes
  tcd->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;
  // Destination - High side of 32 bit register
  tcd->DADDR = (void*)((uint32_t)&I2S1_TDR0 + 2);

  DMAChannel_triggerAtHardwareEvent(DMAMUX_SOURCE_SAI1_TX,TX_DMA_Channel);
  DMAChannel_enable(TX_DMA_Channel);

  I2S1_RCSR |= I2S_RCSR_RE | I2S_RCSR_BCE;
  I2S1_TCSR = I2S_TCSR_TE | I2S_TCSR_BCE | I2S_TCSR_FRDE;

  DMAChannel_attachInterrupt(TX_DMA_Channel,tx_dma_isr_function);
}

static void AudioInputI2S_begin() {

  // We assume this takes the first DMA channel
  RX_DMA_Channel = DMAChannel_begin();

  AudioOutputI2S_config_i2s();

  // RX_DATA0
  CORE_PIN8_CONFIG = 3;  
  IOMUXC_SAI1_RX_DATA0_SELECT_INPUT = 2;
  
  struct TCD* tcd = DMAChannel_getTCD(RX_DMA_Channel);
  // Source address is the high side of the receive buffer
  tcd->SADDR = (void*)((uint32_t)&I2S1_RDR0 + 2);
  tcd->SOFF = 0;
  // Transferring 16 bytes at a time
  tcd->ATTR = DMA_TCD_ATTR_SSIZE(2-1) | DMA_TCD_ATTR_DSIZE(2-1);
  tcd->NBYTES_MLNO = 2;
  tcd->SLAST = 0;
  tcd->DADDR = i2s_rx_buffer;
  tcd->DOFF = 2;
  // Channel linking is disabled, current major iteration count. This is half because
  // each tranfer moves two byes.
  tcd->CITER_ELINKNO = sizeof(i2s_rx_buffer) / 2;
  tcd->DLASTSGA = -sizeof(i2s_rx_buffer);
  // Channel linking is disabled, begin major iteration count
  tcd->BITER_ELINKNO = sizeof(i2s_rx_buffer) / 2;
  // Generate interrupt when major counter completes
  tcd->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;
  DMAChannel_triggerAtHardwareEvent(DMAMUX_SOURCE_SAI1_RX,RX_DMA_Channel);
  
  I2S1_RCSR = I2S_RCSR_RE | I2S_RCSR_BCE | I2S_RCSR_FRDE | I2S_RCSR_FR;
  
  DMAChannel_enable(RX_DMA_Channel);
  DMAChannel_attachInterrupt(RX_DMA_Channel,rx_dma_isr_function);
}

arm_rfft_fast_instance_f32 FFT_Instance;
float32_t LPF_State[64+201-1];

const float32_t LPF_Taps[201] = {43
  -0.0021825804630864584 ,
  0.014877842757177167 ,
  0.002931680946469297 ,
  -0.00045723349767659723 ,
  -0.0016317942082258437 ,
  -0.0005903306753931702 ,
  0.001274251121430665 ,
  0.0016690295810135502 ,
  2.2792272679338813e-05 ,
  -0.001737999488941094 ,
  -0.001443499831885618 ,
  0.0006490351286811217 ,
  0.0020493176319479074 ,
  0.0009907948674766935 ,
  -0.0013458281191702592 ,
  -0.0021422035353976976 ,
  -0.0003346606356218947 ,
  0.001978298268115631 ,
  0.0019676567070739063 ,
  -0.00047294265915399697 ,
  -0.0024548845442167663 ,
  -0.0015083102495319742 ,
  0.0013473230609561578 ,
  0.002685938176531039 ,
  0.0007771219492208337 ,
  -0.002185949263387064 ,
  -0.002604679306198569 ,
  0.00017120661021703253 ,
  0.0028768522492100567 ,
  0.0021690474453561256 ,
  -0.0012523139983631978 ,
  -0.0032995129258213936 ,
  -0.0013811109072176058 ,
  0.002340996722602532 ,
  0.003363820410301346 ,
  0.00028424994868475855 ,
  -0.0033020947339317394 ,
  -0.002995356313329252 ,
  0.001026028307068035 ,
  0.00398625673457576 ,
  0.0021764839275620557 ,
  -0.00241454184750091 ,
  -0.004262525297924699 ,
  -0.0009417361530466481 ,
  0.0037162471314942103 ,
  0.0040311125720681445 ,
  -0.0006200059802246392 ,
  -0.004754101696500414 ,
  -0.0032386824326710146 ,
  0.002367803049410573 ,
  0.005353905993676047 ,
  0.0018880095666554033 ,
  -0.004115995510494866 ,
  -0.005360135739085063 ,
  -4.694614753296915e-05 ,
  0.005648464556233539 ,
  0.0046622199648200914 ,
  -0.002151668262362841 ,
  -0.00669340165147966 ,
  -0.0032295294591903625 ,
  0.004471701312717212 ,
  0.00709960144540818 ,
  0.001078329499274744 ,
  -0.006679600681953499 ,
  -0.006639232537105003 ,
  0.0016623014930160186 ,
  0.008472667336306973 ,
  0.0052035406395631355 ,
  -0.004787253798267356 ,
  -0.009545504324071023 ,
  -0.0027408991203198567 ,
  0.008012236393587682 ,
  0.009591764201242945 ,
  -0.0007256899969452688 ,
  -0.011010584426899851 ,
  -0.008369187761021113 ,
  0.005039687737309171 ,
  0.013354599118410283 ,
  0.005623187528833801 ,
  -0.010009262240112417 ,
  -0.014614556371602059 ,
  -0.0011649532240763502 ,
  0.015342428540737484 ,
  0.01427499126790685 ,
  -0.005228557116139138 ,
  -0.02070087361262177 ,
  -0.011701245151319843 ,
  0.013918143226853705 ,
  0.025736694575548055 ,
  0.005886187307820263 ,
  -0.025802305673919924 ,
  -0.03008130128105119 ,
  0.005346325737187243 ,
  0.04362865279230083 ,
  0.03343801823902569 ,
  -0.029194865114105562 ,
  -0.07930907033041423 ,
  -0.03555794902907878 ,
  0.11625011210872932 ,
  0.29195759086921147 ,
  0.3696161796068215 ,
  0.29195759086921147 ,
  0.11625011210872932 ,
  -0.03555794902907878 ,
  -0.07930907033041423 ,
  -0.029194865114105562 ,
  0.03343801823902569 ,
  0.04362865279230083 ,
  0.005346325737187243 ,
  -0.03008130128105119 ,
  -0.025802305673919924 ,
  0.005886187307820263 ,
  0.025736694575548055 ,
  0.013918143226853705 ,
  -0.011701245151319843 ,
  -0.02070087361262177 ,
  -0.005228557116139138 ,
  0.01427499126790685 ,
  0.015342428540737484 ,
  -0.0011649532240763502 ,
  -0.014614556371602059 ,
  -0.010009262240112417 ,
  0.005623187528833801 ,
  0.013354599118410283 ,
  0.005039687737309171 ,
  -0.008369187761021113 ,
  -0.011010584426899851 ,
  -0.0007256899969452688 ,
  0.009591764201242945 ,
  0.008012236393587682 ,
  -0.0027408991203198567 ,
  -0.009545504324071023 ,
  -0.004787253798267356 ,
  0.0052035406395631355 ,
  0.008472667336306973 ,
  0.0016623014930160186 ,
  -0.006639232537105003 ,
  -0.006679600681953499 ,
  0.001078329499274744 ,
  0.00709960144540818 ,
  0.004471701312717212 ,
  -0.0032295294591903625 ,
  -0.00669340165147966 ,
  -0.002151668262362841 ,
  0.0046622199648200914 ,
  0.005648464556233539 ,
  -4.694614753296915e-05 ,
  -0.005360135739085063 ,
  -0.004115995510494866 ,
  0.0018880095666554033 ,
  0.005353905993676047 ,
  0.002367803049410573 ,
  -0.0032386824326710146 ,
  -0.004754101696500414 ,
  -0.0006200059802246392 ,
  0.0040311125720681445 ,
  0.0037162471314942103 ,
  -0.0009417361530466481 ,
  -0.004262525297924699 ,
  -0.00241454184750091 ,
  0.0021764839275620557 ,
  0.00398625673457576 ,
  0.001026028307068035 ,
  -0.002995356313329252 ,
  -0.0033020947339317394 ,
  0.00028424994868475855 ,
  0.003363820410301346 ,
  0.002340996722602532 ,
  -0.0013811109072176058 ,
  -0.0032995129258213936 ,
  -0.0012523139983631978 ,
  0.0021690474453561256 ,
  0.0028768522492100567 ,
  0.00017120661021703253 ,
  -0.002604679306198569 ,
  -0.002185949263387064 ,
  0.0007771219492208337 ,
  0.002685938176531039 ,
  0.0013473230609561578 ,
  -0.0015083102495319742 ,
  -0.0024548845442167663 ,
  -0.00047294265915399697 ,
  0.0019676567070739063 ,
  0.001978298268115631 ,
  -0.0003346606356218947 ,
  -0.0021422035353976976 ,
  -0.0013458281191702592 ,
  0.0009907948674766935 ,
  0.0020493176319479074 ,
  0.0006490351286811217 ,
  -0.001443499831885618 ,
  -0.001737999488941094 ,
  2.2792272679338813e-05 ,
  0.0016690295810135502 ,
  0.001274251121430665 ,
  -0.0005903306753931702 ,
  -0.0016317942082258437 ,
  -0.00045723349767659723 ,
  0.002931680946469297 ,
  0.014877842757177167 ,
  -0.0021825804630864584 };
  
void setup() {

  Serial.begin(115200);
  delay(1000);
  Serial.println("KC1FSZ");

  arm_status st = arm_rfft_fast_init_f32(&FFT_Instance,1024);
  if (st != ARM_MATH_SUCCESS) {
    Serial.println("Problem with FFT init");
  }

  arm_fir_init_f32(&LPF_Instance,201,(float32_t*)LPF_Taps,LPF_State,64);

  cli();

  // Setup the I2S input
  AudioInputI2S_begin();
  // Setup the I2S output
  AudioOutputI2S_begin();  

  sei();

  delay(1000);
  
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.5);
  sgtl5000_1.inputSelect(AUDIO_INPUT_LINEIN);
  sgtl5000_1.lineInLevel(8);

  CaptureEnabled = true;
  
  Serial.println("setup() done");
}

void doAnalysis() {

  cli();

  // Simple min/max/avg analysis
  float32_t leftMax = 0;
  float32_t leftMin = 0;
  float32_t leftAvg = 0;
  for (unsigned int i = 0; i < 1024; i++) {
    leftAvg += AnalysisBlockL[i];
    if (leftMax == 0 || AnalysisBlockL[i] > leftMax) {
      leftMax = AnalysisBlockL[i];
    }
    if (leftMin == 0 || AnalysisBlockL[i] < leftMin) {
      leftMin = AnalysisBlockL[i];
    }
  }
  leftAvg /= 1024.0;

  // Simple min/max/avg analysis
  float32_t rightMax = 0;
  float32_t rightMin = 0;
  float32_t rightAvg = 0;
  for (unsigned int i = 0; i < 1024; i++) {
    rightAvg += AnalysisBlockR[i];
    if (rightMax == 0 || AnalysisBlockR[i] > rightMax) {
      rightMax = AnalysisBlockR[i];
    }
    if (rightMin == 0 || AnalysisBlockR[i] < rightMin) {
      rightMin = AnalysisBlockR[i];
    }
  }
  rightAvg /= 1024.0;

  sei();

  Serial.print("Stats:");
  Serial.print(" left_min= ");
  Serial.print(leftMin);
  Serial.print(" left_max= ");
  Serial.print(leftMax);
  Serial.print(" left_avg= ");
  Serial.print(leftAvg);
  Serial.print(" right_min= ");
  Serial.print(rightMin);
  Serial.print(" right_max= ");
  Serial.print(rightMax);
  Serial.print(" right_avg= ");
  Serial.print(rightAvg);
  Serial.println();

  cli();
  
  // Spectral analysis
  float32_t leftFftOut[1024];
  arm_rfft_fast_f32(&FFT_Instance,(float32_t*)AnalysisBlockL,leftFftOut,0);
  float32_t rightFftOut[1024];
  arm_rfft_fast_f32(&FFT_Instance,(float32_t*)AnalysisBlockR,rightFftOut,0);

  sei();
  
  // Create magnitude vectors 
  float32_t leftMagOut[512];
  arm_cmplx_mag_f32(leftFftOut,leftMagOut,512);
  float32_t rightMagOut[512];
  arm_cmplx_mag_f32(rightFftOut,rightMagOut,512);

  float32_t leftMaxMag = 0;
  uint32_t leftMaxBucket = 0;
  bool leftMaxValid = false;
  
  for (uint32_t i = 1; i < 512; i++) {
    if (!leftMaxValid || leftMagOut[i] > leftMaxMag) {
      leftMaxMag = leftMagOut[i];
      leftMaxBucket = i;
      leftMaxValid = true;
    }
  }

  float32_t rightMaxMag = 0;
  uint32_t rightMaxBucket = 0;
  bool rightMaxValid = false;
  
  for (uint32_t i = 1; i < 512; i++) {
    if (!rightMaxValid || rightMagOut[i] > rightMaxMag) {
      rightMaxMag = rightMagOut[i];
      rightMaxBucket = i;
      rightMaxValid = true;
    }
  }

  int div = (4410000 / 2) / 512;

  Serial.print("Spectral Max");
  
  Serial.print(" left_mag=");
  Serial.print(leftMaxMag / 1024.0);
  Serial.print(" left_freq=");
  Serial.print(leftMaxBucket * div / 100);

  Serial.print("| right_mag=");
  Serial.print(rightMaxMag / 1024.0);
  Serial.print(" right_freq=");
  Serial.print(rightMaxBucket * div / 100);
  
  Serial.println();
}

volatile long lastDisplay = 0;

void loop() {
  if (millis() - lastDisplay > 2000) {
    lastDisplay = millis();
    if (AnalysisBlockAvailable) {
      doAnalysis();
      AnalysisBlockAvailable = false;   
      // Start on a new one
      CaptureEnabled = true;
    } 
  }
}
