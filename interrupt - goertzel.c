/* 
 * File:   interrupt.c
 * Author: diofa
 *
 * Created/moved on 5.9.19 
 * 
 * 
 */

#include <xc.h>
#include "dsplib_def.h"
#include "dsplib_dsp.h"
#include "mchp_dsp_wrapper.h"
//#include <string.h>
#include "sdrradio32.h"
#include <sys/attribs.h>
#include <sys/kmem.h>

// #include "../SDRradio32.X/../../src_mz/dsp_mz.h"

extern volatile BYTE ADCdone, phase;
extern volatile unsigned int Timer;
extern volatile int32_t *writePointer;
extern volatile uint32_t outBins;
extern volatile unsigned int AGCgain;
extern volatile unsigned int peak,peakM;
extern int32_t g_c,g_cw,g_sw; // Goertzel constants
extern int32_t g_win[FFT_BLOCK_LENGTH]; // Window

#ifndef SLIDING_WINDOW
extern SAMPLE_TYPE __attribute__((coherent)) __attribute__((aligned(16))) srcCV1[FFT_BLOCK_LENGTH],srcCV2[FFT_BLOCK_LENGTH];
#ifdef TEST_SINE_WAVE_MODULATED
extern DWORD __attribute__((coherent)) __attribute__((aligned(16))) testSine[FFT_BLOCK_LENGTH*2],testSine2[FFT_BLOCK_LENGTH*2],testSine3[FFT_BLOCK_LENGTH*2],testSine4[FFT_BLOCK_LENGTH*2];
#endif
#else
extern int32_t __attribute__((coherent)) __attribute__((aligned(16))) srcCV[FFT_BLOCK_LENGTH+SUB_BLOCK_LENGTH];
#endif


void __ISR(_ADC_VECTOR) ADCHandler(void) {

//      LATGINV=0xffffffff;
  
 // Clear the interrupt flag
// INTClearFlag(INT_AD1);
  IFS6bits.ADC0EIF = 0;

 // Do stuff...
  }

void __ISR(_UART1_RX_VECTOR) UART1_ISR(void) {
  
//  LATDbits.LATD4 ^= 1;    // LED to indicate the ISR.
  char curChar = U1RXREG;
  IFS3bits.U1RXIF = 0;  // Clear the interrupt flag!
  }

void __ISR(_DMA0_VECTOR,ipl5AUTO) DMA_ISR(void) {
//  const unsigned int oneSine[4]={128,192,128,64,128};
  SAMPLE_TYPE *p;
  int32_t *pw;
  int32_t z1,z2; // Goertzel status registers
  int32_t I,Q;   // Goertzel output: real, imag, squared magnitude
	int32_t z0;
  int n;

  ADCdone=1;
#ifndef SLIDING_WINDOW
	phase=!phase;   //DMAPPSbits.PPST0
#endif
  
#ifdef TEST_SINE_WAVE_MODULATED
  static WORD cnt;
/*  switch(cnt & 0x0c) {
    case 0x00:
      DCH0SSA = KVA_TO_PA(&testSine);  // transfer source physical address
      break;
    case 0x04:
      DCH0SSA = KVA_TO_PA(&testSine2);  // 
      break;
    case 0x08:
      DCH0SSA = KVA_TO_PA(&testSine3);  // 
      break;
    case 0x0c:
      DCH0SSA = KVA_TO_PA(&testSine4);  // 
      break;
    }*/
  switch(cnt++) {
    case 0:
      DCH0SSA = KVA_TO_PA(&testSine);  // transfer source physical address
      break;
    case 1:
      DCH0SSA = KVA_TO_PA(&testSine2);  // 
      break;
    case 2:
      DCH0SSA = KVA_TO_PA(&testSine3);  // 
      break;
    case 3:
      DCH0SSA = KVA_TO_PA(&testSine4);  // 
      break;
    case 4:
      DCH0SSA = KVA_TO_PA(&testSine3);  // 
      break;
    case 5:
      DCH0SSA = KVA_TO_PA(&testSine2);  // 
      cnt=0;
      break;
    case 6:
      DCH0SSA = KVA_TO_PA(&testSine);  // 
      cnt=0;
      break;
    case 7:
      DCH0SSA = KVA_TO_PA(&testSine4);  // 
      cnt=0;
      break;
    }

 
  
/* cnt &= 7; 
 * 
  for(i=0; i<FFT_BLOCK_LENGTH*2; i+=8) {    // troppo lento a farlo al volo...
    testSine[i]=128;
    testSine[i+2]=128+8*(cnt & 8);
    testSine[i+4]=128;
    testSine[i+6]=128-8*(cnt & 8);
    }*/
    
#endif
#ifdef SLIDING_WINDOW
	if(counter>=SUB_BLOCK_LENGTH) {
		counter=0;
		ADCdone=1;     // 
    if(writePointer >= srcCV+FFT_BLOCK_LENGTH+SUB_BLOCK_LENGTH)
      writePointer=srcCV;
    DCH0DSA = KVA_TO_PA(&srcCV);
    }
  LATEINV=0X0010;     		// LED4 CHECK Timing!		ca. 12mSec (512 SAMP), @140MHz, 11/8/17
#else
  if(phase) {
    DCH0DSA = KVA_TO_PA(&srcCV2);
    p=&srcCV1[0];
    }
  else {
    DCH0DSA = KVA_TO_PA(&srcCV1);
    p=&srcCV2[0];
		}
  
#ifdef PCB2019
  LATESET=0X0004;   // LED2
#else
  LATBINV=0X8000;
#endif

  z1=z2=0; 
  for(n=0,pw=&g_win[0]; n<FFT_BLOCK_LENGTH; n++) {    //~32uS 21/8/21
    int32_t x;
    
 		x = ((((int32_t)*p++) * *pw++) >> SCALING_FACTOR); // windowing / Hamming

    // **** GOERTZEL ITERATION ****
    z0 = x + ((g_c * z1) >> SCALING_FACTOR) - z2; // Goertzel iteration
    z2 = z1; z1 = z0;          // Goertzel status update
    
    }
  
  I = ((g_cw * z1) >> SCALING_FACTOR) - z2; // Goertzel final I
  Q = ((g_sw * z1) >> SCALING_FACTOR); // Goertzel final Q
  outBins = (I * I + Q * Q) >> 16; // magnitude squared
  
 
  OC1RS=outBins / (AGCgain);     // 0..250

#ifdef PCB2019
  LATECLR=0X0004;   // LED2
#else
  LATBINV=0X8000;
#endif

#endif
  
//  DCH0DSA = destinationAddr;    // transfer destination physical address
  
#ifdef PCB2019
  LATEINV=0X0010;     		// LED4 CHECK Timing!		ca. 128uSec (512 SAMP), @200MHz e 2Msamples, 18/8/21; 83uS @3Msamples
#else
//	LATFINV = 0x00000010;		// CHECK Timing!	
#endif

  Timer++;
        
  DCH0INTCLR = _DCH0INT_CHDDIF_MASK /*DCH0INTbits.CHDDIF=0*/;
  IFS4CLR = _IFS4_DMA0IF_MASK /*IFS4bits.DMA0IF=0*/;  // Clear the interrupt flag!
  
  }

#ifdef SSD1309
void __ISR(_DMA1_VECTOR,ipl2AUTO) DMA1_ISR(void) {

  m_SPICSBit=1;

  DCH1INTCLR = _DCH1INT_CHBCIF_MASK /*_DCH1INT_CHDDIF_MASK /*DCH1INTbits.CHDDIF=0*/;
  IFS4CLR = _IFS4_DMA1IF_MASK /*IFS4bits.DMA1IF=0*/;  // Clear the interrupt flag!

  IEC4bits.DMA1IE=0;             // disable DMA channel 1 interrupt
//  DCH1CONbits.CHEN = 0; // turn off DMA channel 1, già sopra messo a one-shot
  
  }
#endif

void __ISR(_TIMER_3_VECTOR,ipl4AUTO) TMR_ISR(void) {
// https://www.microchip.com/forums/m842396.aspx per IRQ priority ecc
  static const unsigned int SINE_WAVE_TABLE[16]={0x8000,0xb0fb,0xda82,0xf641, 0xffff,0xf641,0xda82,0xb0fb,
    0x8000,0x4f04,0x257d,0x09be, 0x0000,0x09be,0x257d,0x4f04};
  static int ptr=0;

//  LED2 ^= 1;      // check timing: 16KHz 25/7/19
  OC1RS   = (SINE_WAVE_TABLE[ptr++]*PWM_MAX)/65536;     // relativo a PR2 del Timer2
  ptr &= 15;
  IFS0CLR = _IFS0_T3IF_MASK;
  }

// ---------------------------------------------------------------------------------------
// declared static in case exception condition would prevent
// auto variable being created
static enum {
	EXCEP_IRQ = 0,			// interrupt
	EXCEP_AdEL = 4,			// address error exception (load or ifetch)
	EXCEP_AdES,				// address error exception (store)
	EXCEP_IBE,				// bus error (ifetch)
	EXCEP_DBE,				// bus error (load/store)
	EXCEP_Sys,				// syscall
	EXCEP_Bp,				// breakpoint
	EXCEP_RI,				// reserved instruction
	EXCEP_CpU,				// coprocessor unusable
	EXCEP_Overflow,			// arithmetic overflow
	EXCEP_Trap,				// trap (possible divide by zero)
	EXCEP_IS1 = 16,			// implementation specfic 1
	EXCEP_CEU,				// CorExtend Unuseable
	EXCEP_C2E				// coprocessor 2
  } _excep_code;

static unsigned int _epc_code;
static unsigned int _excep_addr;

void __attribute__((weak)) _general_exception_handler(uint32_t __attribute__((unused)) code, uint32_t __attribute__((unused)) address) {
  }

void __attribute__((nomips16,used)) _general_exception_handler_entry(void) {
  
	asm volatile("mfc0 %0,$13" : "=r" (_epc_code));
	asm volatile("mfc0 %0,$14" : "=r" (_excep_addr));

	_excep_code = (_epc_code & 0x0000007C) >> 2;

  _general_exception_handler(_excep_code, _excep_addr);

	while (1)	{
		// Examine _excep_code to identify the type of exception
		// Examine _excep_addr to find the address that caused the exception
    }
  }


