/* 
 * File:   interrupt.c
 * Author: diofa
 *
 * Created/moved on 5.9.19 
 * 
 * 
 */

#include <xc.h>
//#include "dsplib_def.h"
//#include "dsplib_dsp.h"
//#include "mchp_dsp_wrapper.h"
//#include <string.h>
#include "sdrradio32.h"
#include <sys/attribs.h>
#include <sys/kmem.h>

#include "../../src_mz/dsp_mz.h"

extern volatile BYTE ADCdone;
extern volatile int16c *sampledData;
static BYTE phase=0;
extern volatile unsigned int Timer,audioValue;
#ifndef SSD1963
extern volatile int32_t *writePointer;
extern volatile uint32_t outBins;
extern volatile unsigned int AGCgain;
extern volatile unsigned int peak,peakM;
extern int32_t g_c,g_cw,g_sw; // Goertzel constants
extern int32_t g_win[FFT_BLOCK_LENGTH]; // Window
#endif

#ifndef SLIDING_WINDOW
extern int16c __attribute__((coherent)) __attribute__((aligned(16))) 
  srcCV1[FFT_BLOCK_LENGTH],srcCV2[FFT_BLOCK_LENGTH],srcCV3[FFT_BLOCK_LENGTH],srcCV4[FFT_BLOCK_LENGTH];
#ifdef TEST_SINE_WAVE
extern WORD __attribute__((coherent)) __attribute__((aligned(16))) testSine[FFT_BLOCK_LENGTH*2];
#endif
#if defined(TEST_SINE_WAVE_MODULATED_AM) || defined(TEST_SINE_WAVE_MODULATED_FM)
extern WORD __attribute__((coherent)) __attribute__((aligned(16))) testSine[FFT_BLOCK_LENGTH*2],testSine2[FFT_BLOCK_LENGTH*2];
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

void __ISR(_DMA0_VECTOR,ipl5SRS) DMA_ISR(void) {
  SAMPLE_TYPE *p;
  int32_t *pw;
	int32_t z0;
  int n;

  ADCdone=1;
#ifndef SLIDING_WINDOW
	phase++;       // (poteva essere DMAPPSbits.PPST0)
  phase &= 3;
#endif
  
#ifdef TEST_SINE_WAVE
#ifdef TEST_SINE_WAVE_MODULATED_AM
  static BYTE cnt;
  switch(cnt++) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
      DCH0SSA = KVA_TO_PA(&testSine);  // 
      break;
    case 5:
    case 6:
    case 7:
    case 8:
      DCH0SSA = KVA_TO_PA(&testSine2);  // 
      break;
    case 9:
      DCH0SSA = KVA_TO_PA(&testSine2);  // 
      cnt=0;
      break;
    }
#elif TEST_SINE_WAVE_MODULATED_FM
  static BYTE cnt;
  switch(cnt++) {
    case 0:
      DCH0SSA = KVA_TO_PA(&testSine);  // transfer source physical address
      break;
    case 1:
      DCH0SSA = KVA_TO_PA(&testSine);  // 
      break;
    case 2:
      DCH0SSA = KVA_TO_PA(&testSine);  // 
      break;
    case 3:
      DCH0SSA = KVA_TO_PA(&testSine);  // 
      break;
    case 4:
      DCH0SSA = KVA_TO_PA(&testSine);  // 
      break;
    case 5:
      DCH0SSA = KVA_TO_PA(&testSine);  // 
      break;
    case 6:
      DCH0SSA = KVA_TO_PA(&testSine);  // 
      break;
    case 7:
      DCH0SSA = KVA_TO_PA(&testSine);  // 
      break;
    case 8:
      DCH0SSA = KVA_TO_PA(&testSine);  // 
      break;
    case 9:
      DCH0SSA = KVA_TO_PA(&testSine2);  // non è il massimo ma proviamo una specie di FM :)
      cnt=0;
      break;
    }
#else
  DCH0SSA = KVA_TO_PA(&testSine);  // transfer source physical address
#endif
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
  switch(phase) {
    case 1:
      DCH0DSA = KVA_TO_PA(&srcCV2);
      sampledData=srcCV1;
      break;
    case 2:
      DCH0DSA = KVA_TO_PA(&srcCV3);
      sampledData=srcCV2;
      break;
    case 3:
      DCH0DSA = KVA_TO_PA(&srcCV4);
      sampledData=srcCV3;
      break;
    case 0:
      DCH0DSA = KVA_TO_PA(&srcCV1);
      sampledData=srcCV4;
      break;
    }
  
#ifndef SSD1963
#ifdef PCB2019
  LATESET=0X0004;   // LED2
#else
  LATBINV=0X8000;
#endif
#endif
 
#ifndef SSD1963
  OC1RS=outBins / (AGCgain);     // 0..250
#else
  
#endif

#ifndef SSD1963
#ifdef PCB2019
  LATECLR=0X0004;   // LED2
#else
  LATBINV=0X8000;
#endif
#else
//  LATGINV=0x0100;   // LED1 90uS 15/8/22 => ~2.8Msamples/sec con Fs=27000000, SAMC=3, clock=50MHz, 8bit (v.)
  // 87uS~ 22/8/22 @3000000
#endif
  
#endif
  
//  DCH0DSA = destinationAddr;    // transfer destination physical address
  
#ifndef SSD1963
#ifdef PCB2019
  LATEINV=0X0010;     		// LED4 CHECK Timing!		ca. 128uSec (512 SAMP), @200MHz e 2Msamples, 18/8/21; 83uS @3Msamples
#else
//	LATFINV = 0x00000010;		// CHECK Timing!	
#endif
#endif

//  Timer++;
        
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

void __ISR(_TIMER_3_VECTOR,ipl4SRS) TMR_ISR(void) {
// https://www.microchip.com/forums/m842396.aspx per IRQ priority ecc
  static const unsigned int SINE_WAVE_TABLE[16]={0x8000,0xb0fb,0xda82,0xf641, 0xffff,0xf641,0xda82,0xb0fb,
    0x8000,0x4f04,0x257d,0x09be, 0x0000,0x09be,0x257d,0x4f04};
  static BYTE ptr=0;

//  LED2 ^= 1;      // check timing: 16KHz 25/7/19
#if 0
  OC1RS   = (SINE_WAVE_TABLE[ptr++]*PWM_MAX)/65536;     // relativo a PR2 del Timer2
  ptr &= 15;
#endif
//  LATGINV=0x0100;   // check timing: 10KHz 22/8/22
  OC3RS   = 250+audioValue;
  Timer++;
  IFS0CLR = _IFS0_T3IF_MASK;
  }

// ---------------------------------------------------------------------------------------
/*******************************************************************************
  Exception Reason Data


  Remarks:
    These global static items are used instead of local variables in the
    _general_exception_handler function because the stack may not be available
    if an exception has occurred.
*/

// Code identifying the cause of the exception (CP0 Cause register).
static unsigned int _excep_code;

// Address of instruction that caused the exception.
static unsigned int _excep_addr;

// Pointer to the string describing the cause of the exception.
static char *_cause_str;

// Array identifying the cause (indexed by _exception_code).
static const char *cause[] = {
    "Interrupt",
    "Undefined",
    "Undefined",
    "Undefined",
    "Load/fetch address error",
    "Store address error",
    "Instruction bus error",
    "Data bus error",
    "Syscall",
    "Breakpoint",
    "Reserved instruction",
    "Coprocessor unusable",
    "Arithmetic overflow",
    "Trap",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved"
  };



// *****************************************************************************
// *****************************************************************************
// Section: Exception Handling
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void _general_exception_handler ( void )

  Summary:
    Overrides the XC32 _weak_ _general_exception_handler.

  Description:
    This function overrides the XC32 default _weak_ _general_exception_handler.

  Remarks:
    Refer to the XC32 User's Guide for additional information.
 */
#include "Adafruit_ST7735.h"
void _general_exception_handler(void) {
    /* Mask off Mask of the ExcCode Field from the Cause Register
    Refer to the MIPs Software User's manual */
  
    _excep_code = (_CP0_GET_CAUSE() & 0x0000007C) >> 2;
    _excep_addr = _CP0_GET_EPC();
    _cause_str  = (char *)cause[_excep_code];
//    SYS_DEBUG_PRINT(SYS_ERROR_FATAL, "\n\rGeneral Exception %s (cause=%d, addr=%x).\n\r",
//                    _cause_str, _excep_code, _excep_addr);


    fillScreen(BLUE);			// 
    setTextColor(BRIGHTRED);
    LCD_XY(0,0);
    gfx_print(_cause_str);    // e _excep_addr ?

    while(1)    {
//        SYS_DEBUG_BreakPoint();
        Nop();
        Nop();
        __delay_ms(2500);
      LED1 ^= 1;
    }
  }



