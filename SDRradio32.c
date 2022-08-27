/* 
 * File:   SDRradio32.c
 * Author: diofa
 *
 * Created on 14 12 2018; 22.7.19 the Liverpool day!
 * 
 * https://github.com/NBitWonder/SDR/blob/master/Firmware/PIC32/main.c

	versione FFT 2022 con Oscillatore Locale, mixer e waterfall/spettro
    https://www.g0kla.com/sdr/tutorials/sdr_tutorial7.php
  */

#include <xc.h>
//#include "dsplib_def.h"
//#include "dsplib_dsp.h"
//#include "mchp_dsp_wrapper.h"
#include "sdrradio32.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <sys/attribs.h>
#include <sys/kmem.h>
//#include <fftc.h>
#include "libmathq15.h"
#include "../../src_mz/dsp_mz.h"

#if defined(ST7735)
#include "Adafruit_ST77xx.h"
#include "Adafruit_ST7735.h"
#elif defined(SSD1309)
#include "Adafruit_SSD1309.h"
#elif defined(SSD1963)
#include "Adafruit_SSD1963.h"
#endif
#include "adafruit_gfx.h"

#include "picojpeg.h"

// #include "../SDRradio32.X/../../src_mz/dsp_mz.h"




// PIC32MZ1024EFE064 Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#pragma config FMIIEN = OFF             // Ethernet RMII/MII Enable (RMII Enabled)
#pragma config FETHIO = OFF             // Ethernet I/O Pin Select (Alternate Ethernet I/O)
#pragma config PGL1WAY = ON             // Permission Group Lock One Way Configuration (Allow only one reconfiguration)
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = OFF            // USB USBID Selection (NOT Controlled by the USB Module)

// per gortzel ecc #warning OCCHIO ANDAVA a 100MHz!!! 2022

// DEVCFG2
/* Default SYSCLK = 200 MHz (8MHz FRC / FPLLIDIV * FPLLMUL / FPLLODIV) */
//#pragma config FPLLIDIV = DIV_1, FPLLMULT = MUL_50, FPLLODIV = DIV_2
#pragma config FPLLIDIV = DIV_1         // System PLL Input Divider (8x Divider)
#pragma config FPLLRNG = RANGE_5_10_MHZ// System PLL Input Range (34-68 MHz Input)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_54       // System PLL Multiplier (PLL Multiply by 128)
#pragma config FPLLODIV = DIV_2        // System PLL Output Clock Divider (32x Divider)
#pragma config UPLLFSEL = FREQ_24MHZ    // USB PLL Input Frequency Selection (USB PLL input is 24 MHz)

// DEVCFG1
#pragma config FNOSC = FRCDIV           // Oscillator Selection Bits (Fast RC Osc w/Div-by-N (FRCDIV))
#pragma config DMTINTV = WIN_127_128    // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN = OFF             // Secondary Oscillator Enable (disable SOSC)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config WDTPS = PS16384          // Watchdog Timer Postscaler (1:16384)
  // circa 6-7 secondi, 24.7.19
#pragma config WDTSPGM = STOP           // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS = NORMAL          // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN = ON              // Watchdog Timer Enable (WDT Enabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window size is 25%)
#pragma config DMTCNT = DMT31           // Deadman Timer Count Selection (2^31 (2147483648))
#pragma config FDMTEN = OFF             // Deadman Timer Enable (Deadman Timer is disabled)

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#ifdef SSD1963
#pragma config ICESEL = ICS_PGx2        // 
#else
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#endif
#pragma config TRCEN = OFF              // Trace Enable (Trace features in the CPU are disabled)
#pragma config BOOTISA = MIPS32         // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FECCCON = OFF_UNLOCKED   // Dynamic Flash ECC Configuration (ECC and Dynamic ECC are disabled (ECCCON bits are writable))
#pragma config FSLEEP = OFF             // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER = PG_ALL          // Debug Mode CPU Access Permission (Allow CPU access to all permission regions)
#pragma config SMCLR = MCLR_NORM        // Soft Master Clear Enable bit (MCLR pin generates a normal system Reset)
#pragma config SOSCGAIN = GAIN_2X       // Secondary Oscillator Gain Control bits (2x gain setting)
#pragma config SOSCBOOST = ON           // Secondary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config POSCGAIN = GAIN_2X       // Primary Oscillator Gain Control bits (2x gain setting)
#pragma config POSCBOOST = ON           // Primary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config EJTAGBEN = NORMAL        // EJTAG Boot (Normal EJTAG functionality)

// DEVCP0
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// SEQ3

// DEVADC0

// DEVADC1

// DEVADC2

// DEVADC3

// DEVADC4

// DEVADC7

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


const char CopyrightString[]= {'S','D','R','R','a','d','i','o','3','2',' ','v',
	VERNUMH+'0','.',VERNUML/10+'0',(VERNUML % 10)+'0',' ','-',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
  '2','7','/','0','8','/','2','2', 0 };

const char Copyr1[]="(C) Dario's Automation 2019-2022 - G.Dar\xd\xa\x0";

BYTE LCD_init(void);         // does whatever is necessary to initialize it
#if defined(ST7735)
#define LCD_cls() clearScreen()          // clear screen
#elif defined(SSD1309)
#define LCD_cls() {	clearDisplay();	display(); }	// serve??sì
#elif defined(SSD1963)
#define LCD_cls() fillScreen(BLACK)          // clear screen
#endif
void LCD_XY(BYTE x, BYTE y);   // position to X/Y character coordinates
#define LCD_Write(s) gfx_print(s)
#define LCD_PutChar(c) cwrite(c)
void LCD_WriteDec(BYTE);      // write 2 digit number
void drawBG(void);
#if defined(SSD1963)
extern const unsigned char tulips[3500L*16+10];
extern const unsigned char skyqueen[3870L*16+9];
uint16_t imageSave[180*5];
#define COLORE_AGO Color565(90,70,55)
#define COLORE_AGO1 Color565(60,50,40)
#define COLORE_AGO2 Color565(100,75,65)
const uint16_t WaterfallColors[9]={ BLUE,LIGHTBLUE,CYAN,BRIGHTCYAN,RED,BRIGHTRED,YELLOW,BRIGHTYELLOW,WHITE};
void drawAgo(int x);
#endif
        
#define Q15(X) \
   ((X < 0.0) ? (int)(0x8000*(X) - 0.5) : (int)(0x7fff*(X) + 0.5)) 
#define Q31(X) \
   ((X < 0.0) ? (int)(0x80000000*(X) - 0.5) : (int)(0x7fffffff*(X) + 0.5)) 
#define q31_float_to_int(x) ( (int) ( (float)(x)*(float)0x7FFFFFFF ) )
#define q15_float_to_int(x) ( (int) ( (float)(x)*(float)0x7FFF ) )
#define	TABLE_LOCALOSC_SIZE (FFT_BLOCK_LENGTH)    // per comodità di calcolo
#warning usare sine_table da q15lib !
q15 oscillator_sinTable[TABLE_LOCALOSC_SIZE],oscillator_cosTable[TABLE_LOCALOSC_SIZE];
int16c local_oscillator_table[TABLE_LOCALOSC_SIZE];
float Complex32ToMag(int32c);
void Complex_multiply(int32c n1,int32c n2,int32c *ret);
void Complex_Real_multiply(q15 n1,int16c n2,int16c *ret);

void prepareLocalOsc(uint8_t, double);
void CCosOscillator_CCosOscillator(DWORD samplesPerSec, int freq, double ampl);
void CSinOscillator_CSinOscillator(DWORD samplesPerSec, int freq, double ampl);
void COscillator_COscillator(DWORD samplesPerSec, int freq, double ampl);
void COscillator_COscillator_(void);
q15 COscillator_nextSample(void);
void initHamming(void);
q15 hammingWindow[FFT_BLOCK_LENGTH];
q15 CFirFilter_filter6(q15 in);
q15 CFirFilter_coeffs5[] = { 
	Q15(0.117119343653851),
	Q15(0.129091809713508),
	Q15(0.133251953115574),
	Q15(0.129091809713508),
	Q15(0.117119343653851)
	};
q15 CFirFilter_xv4[sizeof(CFirFilter_coeffs5)/sizeof(q15)];		// ovvero poi coeffs32; il max possibile!
void CPolyPhaseFilter_CPolyPhaseFilter(DWORD sampleRate, DWORD freq, DWORD decimationRate, DWORD len, double gain);
q15 CPolyPhaseFilter_filter(q15 *in,DWORD numValues);

#define DMA_READY __attribute__((coherent)) __attribute__((aligned(16)))    // usare??
//__attribute__((coherent,aligned(4)))
int16c __attribute__((coherent)) __attribute__((aligned(16))) 
        srcCV1[FFT_BLOCK_LENGTH],srcCV2[FFT_BLOCK_LENGTH],srcCV3[FFT_BLOCK_LENGTH],srcCV4[FFT_BLOCK_LENGTH],
        dstCV[FFT_BLOCK_LENGTH],
        dstCV_BF[FFT_BLOCK_LENGTH/4],filteredCV_BF[FFT_BLOCK_LENGTH/4];   // :16 e *4
//aligned(FFT_BLOCK_LENGTH*2*2)

  
#ifdef TEST_SINE_WAVE
int16c __attribute__((coherent)) __attribute__((aligned(16))) testSine[FFT_BLOCK_LENGTH];
#if defined(TEST_SINE_WAVE_MODULATED_AM) || defined(TEST_SINE_WAVE_MODULATED_FM)
int16c __attribute__((coherent)) __attribute__((aligned(16))) testSine2[FFT_BLOCK_LENGTH];
#endif
#endif


volatile BYTE ADCdone;
volatile int16c *sampledData;
volatile unsigned int Timer;
unsigned int audioValue;

unsigned int AGCgain;
unsigned int peak,peakM,peakF;
#ifdef SSD1963
uint8_t tuneFrequency;
uint32_t getXfromTune(int);
uint32_t getFreqfromTune(int);
uint32_t getLocOscFreqfromTune(int);
uint32_t getFreqfromFFTbin(int);
uint32_t getTunefromFreq(DWORD);
char *getBandMode(BYTE);
char *getModulationMode(BYTE);
BYTE readTouchScreen(uint16_t *xr, uint16_t *yr, uint8_t *zr);
#else
uint8_t demodulatedBin;
#endif
struct SAVED_PARAMETERS configParms;

// http://analoghome.blogspot.com/2016/04/ffts-meet-200mhz-pic32mz-microprocessors.html

//#warning PROVARE Copying fftc to RAM prior to calling this function can be used to improve performance
// bah, vedo tipo un 5%... v. sotto, da 115 a 105-110uS @256 bin
#if (FFT_BLOCK_LENGTH == 64)
#define twiddleFactors fft16c64
#endif
#if (FFT_BLOCK_LENGTH == 128)
#define twiddleFactors fft16c128
#endif
#if (FFT_BLOCK_LENGTH == 256)
#define twiddleFactors fftCoefs  //fft16c256
int16c fftCoefs[FFT_BLOCK_LENGTH];
#define twiddleFactors_bf fftCoefs16  //fft16c16
int16c fftCoefs16[FFT_BLOCK_LENGTH_BF_2];
#endif
#if (FFT_BLOCK_LENGTH == 512 )
#define twiddleFactors fftCoefs  //fft16c512
int16c fftCoefs[FFT_BLOCK_LENGTH];
#define twiddleFactors_bf fftCoefs16  //fft16c16
int16c fftCoefs16[FFT_BLOCK_LENGTH_BF_2];
#endif



#ifdef SSD1963
uint16_t xtouch,ytouch;
uint8_t ztouch;

DWORD jpegBufferLen;
const uint8_t *jpegBufferPtr;
unsigned char pjpeg_need_bytes_callback(unsigned char* pBuf, unsigned char buf_size, unsigned char *pBytes_actually_read, void *pCallback_data) {
  uint32_t n;
  pCallback_data;
   
  n = min(jpegBufferLen, buf_size);
  if(n != buf_size)
    return PJPG_STREAM_READ_ERROR;
  memcpy(pBuf,jpegBufferPtr,n);
  *pBytes_actually_read = (unsigned char)n;
  jpegBufferPtr += n;
  jpegBufferLen -= n;
  return 0;
  }

pjpeg_image_info_t JPG_Info;
#endif


/*
 * 
 */
int main(void) {
  int i;
  uint16_t outBins[FFT_BLOCK_LENGTH/2+1];		// sono metà + 1, pare... NO CAZZATA del tizio :D
//  static int16c *p_cmpx,loc;
  q15 fil;
  char buffer[64];
  static BYTE oldsw=3;
	BYTE waterfallDivider=0;
  int16c scratch[FFT_BLOCK_LENGTH] /*,OutVector[FFT_BLOCK_LENGTH]*/;


/*  configParms.MW_LW=1;
  configParms.AM_FM_SSB=1;
  tuneFrequency=30  +2;
  prepareLocalOsc(tuneFrequency,0.5);

  for(i=0; i<=FFT_BLOCK_LENGTH; i++) {
    int16c *p_comp1,*p_comp2;
    p_comp1=&local_oscillator_table[i];
    if(!(i & 0x80))
      srcCV1[i].re=local_oscillator_table[i].re; //+0x7fff
    else
      srcCV1[i].re=local_oscillator_table[i].re /4; //0 //0x7fff
    p_comp2=&srcCV1[i];
    //srcCV1[i].re/=2;
    srcCV1[i].im=0;
  }
  tuneFrequency=30  +0;
  prepareLocalOsc(tuneFrequency,0.7);
  {
    p_cmpx=(int16c*)&srcCV1; // 
    int32_t audio;
    for(i=0; i<FFT_BLOCK_LENGTH*2; i++) { 
      p_cmpx=(int16c*)&srcCV1[i & 255]; // 
//      p_cmpx->re-=0x7fff;
      loc.re = Q15_MUL(local_oscillator_table[i & 255].re,p_cmpx->re);
      loc.im = Q15_MUL(local_oscillator_table[i & 255].im,p_cmpx->re);
//      audio = libq_q15_Add_q15_q15(loc.re , loc.im);
      audio = loc.re + loc.im;
      fil = CFirFilter_filter6(audio);
      if(i>=4)	{	// il filtro si deve riempire...
        dstCV[i / 16].re=fil;           // ANDREBBE FATTA SOMMA... o no?
        }
      else
        dstCV[i / 16].re=0;
      dstCV[i / 16].im=0;
      p_cmpx++;	
      }
    }
  {  
  DSP_TransformFFT16_setup((int16c*)&fftCoefs, LOG2_BLOCK_LENGTH); // call setup function
	int16c scratch[FFT_BLOCK_LENGTH];
	DSP_TransformFFT16(dstCV,srcCV1,fftCoefs,scratch,LOG2_BLOCK_LENGTH);    // SIMulatore: 75.8uS @256, 34@128, 169.4@512
  for(i=0; i<FFT_BLOCK_LENGTH/2; i++) {      //30uSec con float RICONTROLLARE 2022
    float r1,i1;
    r1=((float)(dstCV[i].re)) / 0x8000L;
    i1=((float)(dstCV[i].im)) / 0x8000L;
//    outBins[i]=1000*sqrt(r1*r1+i1*i1);
    outBins[i]=Q15_POW2(dstCV[i].re)+Q15_POW2(dstCV[i].im);  // ok
//    outBins[i]=libq_q31_Mult2_q15_q15(dstCV[i].re,dstCV[i].re)+libq_q31_Mult2_q15_q15(dstCV[i].im,dstCV[i].im);  // ok a q1.31
    //outBins[i]=q15_add(q15_mul(dstCV[i].re,dstCV[i].re),q15_mul(dstCV[i].im,dstCV[i].im));  // non va... sfora...
//    outBins[i]=(dstCV[i].re*dstCV[i].re+dstCV[i].im*dstCV[i].im) >> 16;   // ok simile a q15_mul
//    outBins[i]=q15_hypot(dstCV[i]);   // pare andare
  }
  }*/
  
  // disable JTAG port
//  DDPCONbits.JTAGEN = 0;

  SYSKEY = 0x00000000;
  SYSKEY = 0xAA996655;    //qua non dovrebbe servire essendo 1° giro (v. IOLWAY
  SYSKEY = 0x556699AA;
  CFGCONbits.IOLOCK = 0;      // PPS Unlock
  SYSKEY = 0x00000000;
#ifndef SSD1963
#ifdef PCB2019
  RPB15Rbits.RPB15R = 4;        // Assign RPB15 as U6TX, pin 30
  U6RXRbits.U6RXR = 2;      // Assign RPB14 as U6RX, pin 29 
  RPD5Rbits.RPD5R = 12;        // Assign RPD5 as OC1, pin 53
#ifdef USA_SPI_HW
  RPG8Rbits.RPG8R = 6;        // Assign RPG8 as SDO2, pin 6
//  SDI2Rbits.SDI2R = 1;        // Assign RPG7 as SDI2, pin 5
#endif
#else
  RPE5Rbits.RPE5R = 1;        // Assign RPE5 as U1TX, pin 1
  U1RXRbits.U1RXR = 3;      // Assign RPD10 as U1RX
  RPD5Rbits.RPD5R = 12;        // Assign RPD5 as OC1, pin 53
//  RPG8Rbits.RPG8R = 6;        // Assign RPG8 as SDO2, pin 6
//  SDI2Rbits.SDI2R = 1;        // Assign RPG7 as SDI2, pin 5
#endif
//	PPSUnLock;
//  PPSInput(1,U1RX, RPB9);  //Assign U1RX to pin RPA09
//  PPSOutput(2,RPC4,U1TX);   //Assign U1TX to pin RPC4

//  PPSOutput(4,RPC4,OC1);   //buzzer 4KHz , qua rimappabile 

#ifdef DEBUG_TESTREFCLK
// test REFCLK
  PPSOutput(4,RPC4,REFCLKO2);   // RefClk su pin 1 (RG15, buzzer)
	REFOCONbits.ROSSLP=1;
	REFOCONbits.ROSEL=1;
	REFOCONbits.RODIV=0;
	REFOCONbits.ROON=1;
	TRISFbits.TRISF3=1;
#endif

#else
  
  RPC14Rbits.RPC14R = 11;     // Assign RPC14 as OC3, pin 48
  RPD3Rbits.RPD3R = 5;        // Assign RPD3 as SDO1, pin 51
  SDI1Rbits.SDI1R = 0;        // Assign RPD2 as SDI1, pin 50
  
#endif
  
  SYSKEY = 0x00000000;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;
  CFGCONbits.IOLOCK = 1;      // PPS Lock
  SYSKEY = 0x00000000;
//	PPSLock;

   // Disable all Interrupts
  __builtin_disable_interrupts();
  
//  SPLLCONbits.PLLMULT=10;
  
  OSCTUN=0;
  OSCCONbits.FRCDIV=0;
  
  // Switch to FRCDIV, SYSCLK=8MHz
  SYSKEY=0xAA996655;
  SYSKEY=0x556699AA;
  OSCCONbits.NOSC=0x00; // FRC
  OSCCONbits.OSWEN=1;
  SYSKEY=0x33333333;
  while(OSCCONbits.OSWEN) {
    Nop();
    }
    // At this point, SYSCLK is ~8MHz derived directly from FRC
 //http://www.microchip.com/forums/m840347.aspx
  // Switch back to FRCPLL, SYSCLK=200MHz
  SYSKEY=0xAA996655;
  SYSKEY=0x556699AA;
  OSCCONbits.NOSC=0x01; // SPLL
  OSCCONbits.OSWEN=1;
  SYSKEY=0x33333333;
  while(OSCCONbits.OSWEN) {
    Nop();
    }
  // At this point, SYSCLK is ~200MHz derived from FRC+PLL
//***
  mySYSTEMConfigPerformance();

    
  
#ifndef SSD1963
  TRISB=0b0000000000010000;       // RB4=AN4 analog input
  TRISC=0b0000000000000000;
#ifdef PCB2019
  TRISD=0x0000000C;       // pulsanti
#else
  TRISD=0x00000000;
#endif
#ifdef PCB2019
  TRISE=0x00000000;   
#else
  TRISE=0x00000000;   
#endif
  TRISF=0x00000000;
  TRISG=0x00000000;
//  TRISH=0x00000000;
  
  ANSELB=0;
  ANSELE=0;
  ANSELG=0;
  
#ifdef PCB2019
  CNPUDbits.CNPUD2=1;   // switch/pulsanti
  CNPUDbits.CNPUD3=1;
  CNPUGbits.CNPUG6=1;   // I2C tanto per
  CNPUGbits.CNPUG8=1;
#else
  CNPUEbits.CNPUE0=1;   // switch/pulsanti
  CNPUEbits.CNPUE1=1;
  CNPUGbits.CNPUG6=1;   // I2C tanto per
  CNPUGbits.CNPUG8=1;
#endif
  
#else
  
  TRISB=0b0000000000000000;
  TRISC=0b0000000000000000;
  TRISD=0b0000000000000000;
  TRISE=0b0000000011110000;     // RE4=AN18 analog input (verificare velocità... i "primari" son tutti su PORTB :(
  TRISF=0b0000000000000000;
  TRISG=0b0000000000000000;
//  TRISH=0x00000000;
  
  ANSELB=0;
  ANSELE=0;
  ANSELG=0;
  
  CNPUEbits.CNPUE5=1;   // switch/pulsanti
  CNPUEbits.CNPUE6=1;
  CNPUEbits.CNPUE7=1;
  
#endif
 
  DSP_TransformFFT16_setup((int16c*)&fftCoefs, LOG2_BLOCK_LENGTH); // call setup function
  DSP_TransformFFT16_setup((int16c*)&fftCoefs16, LOG2_BLOCK_LENGTH_BF);   // rifatto a ogni cambiamento!

  ADCdone=0;
  
  loadSettings();

  AGCgain=50;
  peak=peakM=0;
#ifdef TEST_SINE_WAVE
  tuneFrequency=30;   // per prove, (666KHz loc osc) vs. 675KHz simulatore :)
#else
  tuneFrequency=120;
#endif

  initHamming();
  prepareLocalOsc(tuneFrequency,1.0);
  CPolyPhaseFilter_CPolyPhaseFilter(Fs, 10000, 16, 64, 1);
  
  LED1 = 1;
  
  Timer_Init();
#ifdef SSD1963
  ADC_Init();
  DMA_Init();
  SPI_Init();
#else
  ADC_Init();
  DMA_Init();
#endif
  PWM_Init();
  UART_Init(/*230400L*/ 115200L);

  myINTEnableSystemMultiVectoredInt();
  __delay_ms(1); 



#if defined(ST7735) 
 	LCD_init();

  putsUART1("booting...\r\n");

	gfx_drawRect(0,0,_TFTWIDTH-1,_TFTHEIGHT-1,GREEN);
	gfx_drawRect(1,1,_TFTWIDTH-3,_TFTHEIGHT-3,LIGHTGREEN);

	setTextSize(1);
	setTextColor(BRIGHTGREEN);
	LCD_XY(6,1);
	sprintf(buffer,"SDRradio32 v%u.%02u",VERNUMH,VERNUML);
	LCD_Write(buffer);

	setTextColor(YELLOW);
	LCD_XY(0,3);
	LCD_Write(CopyrightString);
  
	setTextColor(BRIGHTCYAN);
	LCD_XY(5,7);
	sprintf(buffer,"booting...");
	LCD_Write(buffer);
  
#elif defined(SSD1309)
  
 	LCD_init();

  putsUART1("booting...\r\n");

  setRotation(0);
  
	gfx_drawRect(0,0,_TFTWIDTH-1,_TFTHEIGHT-1,WHITE);

	setTextSize(1);
	setTextColor(WHITE);

	LCD_XY(1,1);
	LCD_Write(CopyrightString);
	LCD_XY(1,3);
  sprintf(buffer,"Fs: %uHz",Fs);
	LCD_Write(buffer);
	LCD_XY(1,4);
  sprintf(buffer,"FFT: %u pt",FFT_BLOCK_LENGTH);   // in effetti la metà, in questa versione!
	LCD_Write(buffer);
  
	LCD_XY(6,6);
	LCD_Write("booting...");
  display();
  
#elif defined(SSD1963)
  
  LED2 = 1;
#ifndef USING_SIMULATOR
 	LCD_init();

  putsUART1((unsigned int *)"booting...\r\n");

  setRotation(0);
  
	gfx_drawRect(0,0,_TFTWIDTH-1,_TFTHEIGHT-1,GREEN);
	gfx_drawRect(1,1,_TFTWIDTH-3,_TFTHEIGHT-3,LIGHTGREEN);

	setTextSize(2);
	setTextColor(WHITE);
	LCD_XY(1,1);
	LCD_Write((char *)CopyrightString);
	setTextColor(WHITE);
	LCD_XY(1,4);
  sprintf(buffer,"Fs: %uHz",Fs);
	LCD_Write(buffer);
	LCD_XY(1,5);
  sprintf(buffer,"FFT: %u pt",FFT_BLOCK_LENGTH);   // in effetti la metà, in questa versione!
	LCD_Write(buffer);
  
	LCD_XY(30,7);
	LCD_Write("booting...");
	setTextSize(1);
	setTextColor(BRIGHTGREEN);
#endif
#endif

  __delay_ms(2000);
/*  for(;;) {
    LATEINV=0xffffffff;   // test timing, 500Hz con PLL=51 e FCY 205000000; 3.9.19
    __delay_us(1000);
    }*/

#if defined(ST7735) || defined(SSD1309)
  clearDisplay(); //LCD_cls();
	LCD_XY(1,1);
  sprintf(buffer,"Frequenza: %uKHz",((Fs/(FFT_BLOCK_LENGTH/2 /* per la mezza FFT! */))*demodulatedBin)/1000);
	LCD_Write(buffer);
  drawLine(0,22,127,22,WHITE);
  display();
#elif defined(SSD1963)
#ifndef USING_SIMULATOR
  LCD_cls();
	LCD_XY(1,1);
  sprintf(buffer,"Frequenza: %uKHz",((Fs/(FFT_BLOCK_LENGTH/2 /* per la mezza FFT! */))*tuneFrequency)/1000);
	LCD_Write(buffer);
  drawLine(0,22,127,22,WHITE);
#endif
#endif

  
#ifdef TEST_SINE_WAVE
  int cnt,baseV;
  
  baseV=0x0000;   // 0b 00000000 00000000 00000100 cc000000
                  // (0b 00000000 00000000 0000nnnn nnnn0000  questo è come arriva l'ADC)
                  // 0b sddddddd dddddddd  questo è come arriva l'ADC
  cnt=0x3000;
  for(i=0; i<FFT_BLOCK_LENGTH; i+=4) {    // 
//  75,0,150,0,75,0,0,0,    // 0 (+ complex))
    testSine[i].re=baseV;    // 12 bit right-justified di cui i 4 bassi non usati (v. pdf))
    testSine[i].im=0;
    testSine[i+1].re=baseV+cnt;
    testSine[i+1].im=0;
    testSine[i+2].re=baseV;
    testSine[i+2].im=0;
    testSine[i+3].re=baseV-cnt;
    testSine[i+3].im=0;

  }
#warning usare COscillator!
#ifdef TEST_SINE_WAVE_MODULATED_AM
  cnt=0x500;
  for(i=0; i<FFT_BLOCK_LENGTH; i+=4) {    // 
    testSine2[i].re=baseV;
    testSine2[i].im=0;
    testSine2[i+1].re=baseV+cnt;
    testSine2[i+1].im=0;
    testSine2[i+2].re=baseV;
    testSine2[i+2].im=0;
    testSine2[i+3].re=baseV-cnt;
    testSine2[i+3].im=0;
    }
#endif
#ifdef TEST_SINE_WAVE_MODULATED_FM
  cnt=0x2000;
  for(i=0; i<FFT_BLOCK_LENGTH; i+=8) {    // 
    testSine2[i].re=baseV;
    testSine2[i].im=0;
    testSine2[i+1].re=baseV;
    testSine2[i+1].im=0;
    testSine2[i+2].re=baseV+cnt;
    testSine2[i+2].im=0;
    testSine2[i+3].re=baseV+cnt;
    testSine2[i+3].im=0;
    testSine2[i+4].re=baseV;
    testSine2[i+4].im=0;
    testSine2[i+5].re=baseV;
    testSine2[i+5].im=0;
    testSine2[i+6].re=baseV-cnt;
    testSine2[i+6].im=0;
    testSine2[i+7].re=baseV-cnt;
    testSine2[i+7].im=0;
    }
#endif
#endif
  
redraw:
#ifndef USING_SIMULATOR
  if(!configParms.waterfall) {
    drawJPEG(0,0,800,450 /*480*/,skyqueen,sizeof(skyqueen));
    readArea(getXfromTune(tuneFrequency)-2,5,getXfromTune(tuneFrequency)+2,175,imageSave);
    drawAgo(getXfromTune(tuneFrequency));
    }
  else {
    gfx_drawRect(66, 100, 649, 352, BRIGHTGREEN);
    }
#endif

  while(1) {
    static int divider=0;
    UGRAPH_COORD_T x,y,x1,y1;

    ClrWdt();

//  LED1 ^= 1;		// CHECK Timing!	200-300nS a vuoto, 15/8/22; 
  
    
    if(ADCdone) {
      int16c *p_cmpx;
      
  		if(configParms.waterfall) {
  LATGSET=0x0100;   // LED1 
  

#ifdef DEBUG_PRINT_VALUES          
					p_cmpx=(int16c *)sampledData; // 
            LCD_cls();
            LCD_XY(0,0);
            
					for(i=0; i<FFT_BLOCK_LENGTH; i++) {   // Convert the Real input sample array to a Complex input sample array  
            
            
            
sprintf(buffer,"%u: %04x-%04x ",i,p_cmpx->re,p_cmpx->im);
LCD_Write(buffer);
if(!(i & 7))
  LCD_PutChar('\n');
						p_cmpx++;	
						}
 __delay_ms(2000);
//      while(!SW1)
//        ClrWdt();
#endif

          
          uint16_t *p_int;
//          DSP_TransformWindow_Hamm16(OutVector, sampledData, LOG2_BLOCK_LENGTH);    // circa 35uS 22/8/22
          // mmm NON VA bene, quella vuole INT e non COMPLEX!
					p_cmpx=(int16c*)sampledData; // 
          p_int=(uint16_t*)&hammingWindow;
          for(i=0; i<FFT_BLOCK_LENGTH; i++) { 
            p_cmpx->re = Q15_MUL(p_cmpx->re,*p_int++);
            p_cmpx++;
            }
          DSP_TransformFFT16(dstCV,(int16c*)/*OutVector */sampledData,twiddleFactors,scratch,LOG2_BLOCK_LENGTH/*    -1*/);    // 110uS 256bin; 65us 128bin

//	LATGCLR=0x0100;   // LED1 115uS 22/8/22 => 3Msamples/sec
  
#ifdef DEBUG_PRINT_VALUES          
            LCD_cls();
            LCD_XY(0,0);
#endif
					p_cmpx=(int16c*)&dstCV; // 
          p_int=(uint16_t*)&outBins;
          i=FFT_BLOCK_LENGTH/2;       //30uSec con float RICONTROLLARE 2022

#warning SI POTREBBE LASCIARE I VALORI IN DSTCV E POI FARE I QUADRATI AL MOMENTO DEL PLOTTAGGIO sono ~15uS...
          while(i--) {
/*            float r1,i1;
	          r1=((float)(dstCV[i].real)) / 0x80000000L;
	          i1=((float)(dstCV[i].imag)) / 0x80000000L;
	          v1=2*4*sqrt(r1*r1+i1*i1);*/
            *p_int++ = Q15_POW2(p_cmpx->re)+Q15_POW2(p_cmpx->im);
//            outBins[i]=/*libq_q31_Mult2_q15_q15*/Q15_POW2(p_cmpx->re)+/*libq_q31_Mult2_q15_q15*/Q15_POW2(p_cmpx->im);
            //outBins[i]=q15_hypot(dstCV[i]);   // 
            // [esce 0x01c7 con la test wave modulata, bin 64]
            p_cmpx++;
            
#ifdef DEBUG_PRINT_VALUES          
sprintf(buffer,"%u: %04x-%04x-%04x ",i,dstCV[i].re,dstCV[i].im,q15_mul(dstCV[i].re,dstCV[i].re)+q15_mul(dstCV[i].im,dstCV[i].im));
LCD_Write(buffer);
if(!(i & 7))
  LCD_PutChar('\n');
#endif
            

/*
//            v1=q31_sqrt(q31_mul(dstCV[i].real,dstCV[i].real)+q31_mul(dstCV[i].imag,dstCV[i].imag));
            // 300uS 16/8 con q31_sqrt ecc, ma credo si possa evitare la sqrt
            v1=q31_mul(dstCV[i].real,dstCV[i].real)+q31_mul(dstCV[i].imag,dstCV[i].imag);
            //100us usando q31_mul con shift
            v1 *= (2*4);*/
						}

//          audioValue=500;       // tanto per :)
  LATGCLR=0x0100;   // LED1 130uS 22/8/22 => ~3Msamples/sec (salta 1 sample ogni 2, giustamente)
                    // 160uS con hamming mia 25/8/22

 #ifdef DEBUG_PRINT_VALUES          
__delay_ms(2000);
//      while(!SW1)
//        ClrWdt();
#endif

        }
        
			else {    // no waterfall
          int16c loc,*p_loc;
          static q15 audio;   // static per SSB/pass..

          //DSP_TransformWindow_Hamm16(int16_t * OutVector, int16_t * InVector, int N)
//            p_cmpx->re = Q15_MUL(p_cmpx->re,hammingWindow[i]);

  LATGSET=0x0100;   // LED1 
					p_cmpx=(int16c*)sampledData; // 
          p_loc=local_oscillator_table;
          static BYTE pass=0;
          switch(configParms.AM_FM_SSB) {
            q15 filteredCV_BF2[16];
            BYTE j;
            case 1:   // AM e FM
            case 2:
              for(i=0; i<FFT_BLOCK_LENGTH; i+=16) { 
                for(j=0; j<16; j++) { 
                  loc.re = Q15_MUL(p_loc->re,p_cmpx->re);
                  loc.im = Q15_MUL(p_loc->im,p_cmpx->re);
                  filteredCV_BF2[j]=Q15_ADD(loc.re,loc.im);
                  p_cmpx++;
                  p_loc++;
                  }
                fil=CPolyPhaseFilter_filter(filteredCV_BF2,16);
            
                filteredCV_BF[i / 16].re=fil;           // ANDREBBE FATTA SOMMA... o no?
                filteredCV_BF[i / 16].im=0;
                }
              break;
            case 3: // SSB
            case 4:
            case 5:
              {
              for(i=0; i<FFT_BLOCK_LENGTH; i+=16) { 
                // I = REAL e Q = IMAG :)
                for(j=0; j<16; j++) { 
                  loc.re = Q15_MUL(p_loc->re,p_cmpx->re);
                  loc.im = Q15_MUL(p_loc->im,p_cmpx->re);
                  filteredCV_BF2[j]=Q15_ADD(loc.re,loc.im);
                  p_cmpx++;	
                  p_loc++;
                  }
                fil=CPolyPhaseFilter_filter(filteredCV_BF2,16);            
                filteredCV_BF[pass*16 + i/16].re=fil;           // ANDREBBE FATTA SOMMA... o no?
                filteredCV_BF[pass*16 + i/16].im=0;
                }
              pass++;
              pass &= 3;
              }
              break;
            }

//  LATGCLR=0x0100;   // LED1 210uS FIN QUA! 17/8/22 => ~2.8Msamples/sec

          switch(configParms.AM_FM_SSB) {
            case 1:
#if 0
//            LCD_cls();
//            LCD_XY(0,0);
            
    					p_cmpx=(int16c*)&filteredCV_BF; // 
              audio=0;
        			for(i=0; i<16; i++) {
                audio += p_cmpx->re>0 ? p_cmpx->re : 0;
//                audio += p_cmpx->re;
//sprintf(buffer,"%u: %04x-%04x ",i,p_cmpx->re,p_cmpx->im);
//LCD_Write(buffer);
//if(!(i & 7))
//  LCD_PutChar('\n');
                p_cmpx++;	
                }
//              if(audio<0)
//                audio=0;
              audio /= 32;
//    					audio=filteredCV_BF[0].re+filteredCV_BF[1].re;
//sprintf(buffer," audio: %u ",audio);
//LCD_Write(buffer);
//      while(!SW2)
//        ClrWdt();
#else
    					DSP_TransformFFT16(dstCV_BF,filteredCV_BF,twiddleFactors_bf,scratch,LOG2_BLOCK_LENGTH_BF);    // 110uS 256bin; 65us 128bin
              audio=Q15_POW2(dstCV_BF[1].re)+Q15_POW2(dstCV_BF[1].im);     // 0..1000  ??
#endif
              break;
            case 2:
    					DSP_TransformFFT16(dstCV_BF,filteredCV_BF,twiddleFactors_bf,scratch,LOG2_BLOCK_LENGTH_BF);    // 4uS 16bin 21/8/22
              audio=q15_atan2(dstCV_BF[1].im,dstCV_BF[1].re);     //fare arctan(Q/I) per fase....
//              audio=q15_abs(audio);
              // continua a andare in negativo... strano
//              audio *= 10;
              audio /= 50;    // credo 0..32767 qua; si potrebbe anche fare 32767- 
              break;
            case 3:
              if(!pass) {   // solo ogni 4...
                DSP_TransformFFT16(dstCV_BF,filteredCV_BF,twiddleFactors_bf,scratch,LOG2_BLOCK_LENGTH_BF_2);    // 110uS 256bin; 65us 128bin
                audio=Q15_POW2(dstCV_BF[3].re)+Q15_POW2(dstCV_BF[3].im);    // circa 3KHz a bin (3000000/16/2/64/2)
//                audio *= 10;
                }
              break;
            case 4:
              if(!pass) {
                DSP_TransformFFT16(dstCV_BF,filteredCV_BF,twiddleFactors_bf,scratch,LOG2_BLOCK_LENGTH_BF_2);    // 110uS 256bin; 65us 128bin
                audio=Q15_POW2(dstCV_BF[1].re)+Q15_POW2(dstCV_BF[1].im)+Q15_POW2(dstCV_BF[2].re)+Q15_POW2(dstCV_BF[2].im);
//                audio *= 10;
                }
              break;
            case 5:
              if(!pass) {
                DSP_TransformFFT16(dstCV_BF,filteredCV_BF,twiddleFactors_bf,scratch,LOG2_BLOCK_LENGTH_BF_2);    // 110uS 256bin; 65us 128bin
                audio=Q15_POW2(dstCV_BF[4].re)+Q15_POW2(dstCV_BF[4].im)+Q15_POW2(dstCV_BF[5].re)+Q15_POW2(dstCV_BF[5].im);
//                audio *= 10;
                }
              break;
            default:
              break;
            }

          audioValue=audio/(AGCgain/3);   //0..500 (+500) 27/8/22
          if(audio > 0) {
            if(audio > peak) {
              peak=q15_abs(audio);
              }
            }

  LATGCLR=0x0100;   // LED1 150uS 22/8/22 => ~3Msamples/sec con filtro, decimazione e seconda DSP @16
                    // 140uS con semplice somma al posto di DSP
      // 105uS AM unrolled, 115uS FM, 130uS le SSB
	// 100uS AM con poly-4x16, 105 FM, 130 SSB
  // 90uS AM (con FFT) usando puntatori in poly; 85 FM; 90 SSB

      }   // else no waterfall
              

      ADCdone=0;
      }

    
#ifndef SSD1963
    if(Timer>(2*(Fs/10/FFT_BLOCK_LENGTH))) {   // Timer incrementa ogni 128uS @1Msamples, 128 FFT, voglio ~100mS
#else
//    if(Timer>((Fs/10/FFT_BLOCK_LENGTH))) {   // Timer incrementa ogni 90uS @2.7Msamples, 256 FFT, voglio ~100mS
    if(Timer>=1000) {   // Timer incrementa ogni 100uS
#endif
      divider++;
      
      LED2 ^= 1;      //100mS 25/8/22
      
#if defined(SSD1963)

      if(configParms.waterfall) {
        waterfallDivider++;
        if(waterfallDivider>=5) {
          waterfallDivider=0;
#ifndef USING_SIMULATOR
//    gfx_drawRect(66, 100, 648, 352, BRIGHTGREEN);
          scrollArea(67, 102, 713, 450, 0, -1);
          // TOGLIERE 0 poi!
          peakM=0;
          for(i=0; i<FFT_BLOCK_LENGTH/2; i++) {
            drawFastHLine(68+i*5,450,4,WaterfallColors[min(outBins[i] /*>> 1*/, 8)]);
            if(outBins[i]>peakM) {
              peakM=outBins[i];
              peakF=getFreqfromFFTbin(i);
              
              tuneFrequency=getTunefromFreq(peakF);     // auto-tuning, utile per ora!
              
              }
            // [in effetti è lento, meglio gestire un array di diciamo 128W x 300H byte e shiftare/plottare..]
            }
          audioValue=peakM;       // tanto per :)
#endif
          }
        }
      else {         // plottare spettro, 16 barre qua...
        peakM += peak;
        peakM /= 2;
        if(configParms.spectrum) {
          waterfallDivider++;
          if(waterfallDivider>=5) {
            waterfallDivider=0;
            fillRect(600,465,799,479,BLACK);
            switch(configParms.AM_FM_SSB) {
              case 1:
      					DSP_TransformFFT16(dstCV_BF,filteredCV_BF,twiddleFactors_bf,scratch,LOG2_BLOCK_LENGTH_BF);
                // non la faccio sopra..
              case 2:
                // TOGLIERE 0 poi!
                for(i=0; i<FFT_BLOCK_LENGTH_BF/2; i++) {
                  int32_t j;
                  j=Q15_POW2(dstCV_BF[i].re)+Q15_POW2(dstCV_BF[i].im);
                  if(j<5)
                    j=5;
                  if(j>50)
                    j=50;
                  drawFastVLine(602+i*5,478-j/5,j/5,ORANGE);
                  }
                break;
              case 3:
              case 4:
              case 5:
                // TOGLIERE 0 poi!
                for(i=0; i<FFT_BLOCK_LENGTH_BF_2/2; i++) {
                  int32_t j;
                  j=Q15_POW2(dstCV_BF[i].re)+Q15_POW2(dstCV_BF[i].im);
                  if(j<4)
                    j=4;
                  if(j>40)
                    j=40;
                  drawFastVLine(602+i*2,478-j/4,j/4,ORANGE);
                  }
                break;
              }
            }
          }
        }

      if(!SW1) {
        if(!configParms.waterfall) {
          putArea(getXfromTune(tuneFrequency)-2,5,getXfromTune(tuneFrequency)+2,175,imageSave);
          }
        if(tuneFrequency > (configParms.MW_LW ? 0 : 0))     // tarare!
          tuneFrequency--;
        prepareLocalOsc(tuneFrequency,1.0);
        if(!configParms.waterfall) {
          readArea(getXfromTune(tuneFrequency)-2,5,getXfromTune(tuneFrequency)+2,175,imageSave);
          drawAgo(getXfromTune(tuneFrequency));
          }
        }
      if(!SW2) {
        if(!configParms.waterfall) {
          putArea(getXfromTune(tuneFrequency)-2,5,getXfromTune(tuneFrequency)+2,175,imageSave);
          }
        if(tuneFrequency < (configParms.MW_LW ? 245 : 245))     // TARARE!
          tuneFrequency++;
        prepareLocalOsc(tuneFrequency,1.0);
        if(!configParms.waterfall) {
          readArea(getXfromTune(tuneFrequency)-2,5,getXfromTune(tuneFrequency)+2,175,imageSave);
          drawAgo(getXfromTune(tuneFrequency));
          }
        }
      if(!SW3) {
        
        if(!configParms.waterfall) {
        configParms.AM_FM_SSB++;
        if(configParms.AM_FM_SSB>5)
          configParms.AM_FM_SSB=1;
          switch(configParms.AM_FM_SSB) {
            case 1:
            case 2:
              DSP_TransformFFT16_setup((int16c*)&fftCoefs16, LOG2_BLOCK_LENGTH_BF);
              break;
            case 3:
            case 4:
            case 5:
              DSP_TransformFFT16_setup((int16c*)&fftCoefs16, LOG2_BLOCK_LENGTH_BF_2);
              break;
            }

        }

      	setTextColor(BRIGHTCYAN);
        LCD_cls();
	setTextSize(2);
        LCD_XY(1,1);
        sprintf(buffer,"Frequency: %uKHz",getFreqfromTune(tuneFrequency)/1000);
        LCD_Write(buffer);
        LCD_XY(25,1);
        LCD_Write(getBandMode(configParms.MW_LW));
        LCD_XY(32,1);
        LCD_Write(getModulationMode(configParms.AM_FM_SSB));
        LCD_XY(1,2);
        sprintf(buffer,"Signal: %u",AGCgain);
        LCD_Write(buffer);
        LCD_XY(1,3);
        sprintf(buffer,"Peak: %u",peakM);
        LCD_Write(buffer);
        LCD_XY(1,4);
        sprintf(buffer,"Volume: %u",  100);
        LCD_Write(buffer);
        LCD_XY(34,4);
        sprintf(buffer,"PR1: %u",  PR1);
        LCD_Write(buffer);
        LCD_XY(44,4);
        sprintf(buffer,"PRECON: %08X",  PRECON);
        LCD_Write(buffer);
        
setTextSize(1);
        drawLine(0,100,799,100,WHITE);
        
        __delay_ms(1500);
        configParms.waterfall=!configParms.waterfall;
        if(configParms.waterfall) {
        	setTextColor(ORANGE);
          for(i=0; i<=FFT_BLOCK_LENGTH/2 /16; i++) {
            LCD_XY(((i+1)*13)-3,57);
            sprintf(buffer,"%uKHz",getFreqfromFFTbin(i*16)/1000);
            LCD_Write(buffer);
            }
          }
        goto redraw;
        }
      
#else
      if(!SW2) {
        if(demodulatedBin > 0)
          demodulatedBin--;
        }
      if(!SW1) {
        if(demodulatedBin < 127)
        demodulatedBin++;
        }
#endif
      
updateLCD:
#if defined(ST7735) || defined(SSD1309)
        clearDisplay(); //LCD_cls();
        LCD_XY(1,1);
        sprintf(buffer,"Frequency: %uKHz",((Fs/(FFT_BLOCK_LENGTH/2 /* per la mezza FFT! */))*demodulatedBin)/1000);
        LCD_Write(buffer);
        LCD_XY(1,2);
        sprintf(buffer,"Signal: %u",AGCgain);
        LCD_Write(buffer);
        LCD_XY(1,4);
        sprintf(buffer,"Peak: %u",peakM);
        LCD_Write(buffer);
        drawLine(0,31,127,31,WHITE);
        displayDMA();
#elif defined(SSD1963)
        divider++;
        if(divider>5) {
          divider=0;
      	setTextColor(BRIGHTGREEN);
        fillRect(1,470,499,478,BLACK);
  setTextSize(1);
        LCD_XY(1,59);
//        sprintf(buffer,"Signal: %u, %X",AGCgain,ADCDATA18);
        sprintf(buffer,"Signal: %u",AGCgain);
        LCD_Write(buffer);
	      LCD_XY(25,59);
        sprintf(buffer,"Peak: %u",peakM);
        LCD_Write(buffer);
	      LCD_XY(42,59);
        if(configParms.waterfall)
          sprintf(buffer,"Fpeak: %uKHz",peakF/1000);
        else
          sprintf(buffer,"F: %u (loc: %u)",getFreqfromTune(tuneFrequency),getLocOscFreqfromTune(tuneFrequency));
        LCD_Write(buffer);
	      LCD_XY(72,59);
        LCD_Write(getBandMode(configParms.MW_LW));
	      LCD_XY(76,59);
        LCD_Write(getModulationMode(configParms.AM_FM_SSB));
        }
#endif
        
        peak=0;
#ifndef SSD1963
        AGCgain += (peakM/10000);  //circa 80000 senza segnale, circa 600000 con 
        AGCgain /= 2;
        if(AGCgain<5)
          AGCgain=5;
#else
        AGCgain += ((peakM+50)/100);  //circa 0 senza segnale, circa 4000 con testSine=0x500-0x3000 (diciamo 40%)
        AGCgain /= 2;
        if(AGCgain<10)
          AGCgain=10;
#endif


        readTouchScreen(&xtouch,&ytouch,&ztouch);
        
      oldsw=(SW1 ? 0 : 1) | (SW2 ? 0 : 2);
      Timer=0;
      }

    }

  return EXIT_SUCCESS;
  }


uint32_t getXfromTune(int n) {

  return 100+(tuneFrequency*5)/2;
  }

uint32_t getFreqfromTune(int n) {

  if(configParms.MW_LW)
    return ((DWORD)n)*4500+540000UL;
  else
    return ((DWORD)n)*1600+160000UL;
  }

uint32_t getLocOscFreqfromTune(int n) {

  if(configParms.MW_LW) {
    switch(configParms.AM_FM_SSB) {
      }
    return ((DWORD)n)*4500+540000UL-9000;
    }
  else {
    switch(configParms.AM_FM_SSB) {
      }
    return ((DWORD)n)*1600+160000UL-9000;
    }
  }

uint32_t getTunefromFreq(DWORD n) {

  if(configParms.MW_LW)
    return (n-540000UL)/4500;
  else
    return (n-160000UL)/4500;
  }

uint32_t getFreqfromFFTbin(int n) {

  if(configParms.MW_LW)
    return (Fs/2/(FFT_BLOCK_LENGTH/2))*n;
  // direi che qua non cambia nulla..
  else
    return (Fs/2/(FFT_BLOCK_LENGTH/2))*n;
  }

char *getBandMode(BYTE m) {
  switch(m) {
    case 0:
      return "LW";
      break;
    case 1:
      return "MW";
      break;
    }
  return "";
  }
char *getModulationMode(BYTE m) {
  
  switch(m) {
    case 0:
      return "n/a";
      break;
    case 1:
      return "AM";
      break;
    case 2:
      return "FM";
      break;
    case 3:
      return "SSB";
      break;
    case 4:
      return "LSB";
      break;
    case 5:
      return "USB";
      break;
    }
  return "";
  }

// -----------------------------------------------------------------------------
enum CACHE_MODE {
  UNCACHED=0x02,
  WB_WA=0x03,
  WT_WA=0x01,
  WT_NWA=0x00,
/* Cache Coherency Attributes */
//#define _CACHE_WRITEBACK_WRITEALLOCATE      3
//#define _CACHE_WRITETHROUGH_WRITEALLOCATE   1
//#define _CACHE_WRITETHROUGH_NOWRITEALLOCATE 0
//#define _CACHE_DISABLE                      2
  };
void mySYSTEMConfigPerformance(void) {
  unsigned int PLLIDIV;
  unsigned int PLLMUL;
  unsigned int PLLODIV;
  double CLK2USEC;
  unsigned int SYSCLK;
  unsigned char PLLODIVVAL[]={
    2,2,4,8,16,32,32,32
    };
	unsigned int cp0;

  PLLIDIV=SPLLCONbits.PLLIDIV+1;
  PLLMUL=SPLLCONbits.PLLMULT+1;
  PLLODIV=PLLODIVVAL[SPLLCONbits.PLLODIV];

  SYSCLK=(FOSC*PLLMUL)/(PLLIDIV*PLLODIV);
  CLK2USEC=SYSCLK/1000000.0f;

  SYSKEY = 0x00000000;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;

  if(SYSCLK<=60000000L)
    PRECONbits.PFMWS=0;
  else if(SYSCLK<=120000000L)
    PRECONbits.PFMWS=1;
  else if(SYSCLK<=200000000L)
    PRECONbits.PFMWS=2;
  else if(SYSCLK<=220000000L)
    PRECONbits.PFMWS=3;
  else if(SYSCLK<=252000000L)
    PRECONbits.PFMWS=4;
  else
    PRECONbits.PFMWS=7;

  PRECONbits.PFMSECEN=0;    // non c'è nella versione "2019" ...
  PRECONbits.PREFEN=0x1;

  SYSKEY = 0x0;

  // Set up caching
  cp0 = _mfc0(16, 0);
  cp0 &= ~0x07;
  cp0 |= 0b011 /*WB_WA*/; // K0 = Cacheable, non-coherent, write-back, write allocate
  _mtc0(16, 0, cp0);  

  SYSKEY = 0x00000000;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;
#ifdef TEST_SINE_WAVE
  CFGCONbits.DMAPRI = 1;      // DMA priority
#endif
  SYSKEY = 0x00000000;

  }

void myINTEnableSystemMultiVectoredInt(void) {

  PRISS = 0x76543210;
  INTCONSET = _INTCON_MVEC_MASK /*0x1000*/;    //MVEC
  asm volatile ("ei");
  //__builtin_enable_interrupts();
  }

/* CP0.Count counts at half the CPU rate */
#define TICK_HZ (CPU_HZ / 2)

void __attribute__((used)) __delay_ns(unsigned int nsec) {
  unsigned int tWait, tStart;

#ifndef USING_SIMULATOR
  tWait=((GetSystemClock()/1000)*nsec)/2000000UL;
  tStart=_mfc0(9,0);
  while((_mfc0(9,0)-tStart)<tWait)
    ClrWdt();        // wait for the time to pass
#endif
  }

void __attribute__((used)) __delay_us(unsigned int usec) {
  unsigned int tWait, tStart;

#ifndef USING_SIMULATOR
  tWait=(GetSystemClock()/2000000)*usec;
  tStart=_mfc0(9,0);
  while((_mfc0(9,0)-tStart)<tWait)
    ClrWdt();        // wait for the time to pass
#endif
  }

void __attribute__((used)) __delay_ms(unsigned int ms) {
  
#ifndef USING_SIMULATOR
  while(ms--)
    __delay_us(1000);
#endif
  }


void Timer_Init(void) {

  T2CON=0;
  T2CONbits.TCS = 0;                  // clock from peripheral clock
  T2CONbits.TCKPS = 0;                // 1:1 prescale
  T2CONbits.T32 = 0;                  // 16bit
#ifdef SSD1963
  PR2 = PERIPHERAL_CLOCK_HZ/100000;     // rollover every n clocks; 1000 = 100KHz
#else
  PR2 = 2000;                         // rollover every n clocks; 2000 = 50KHz
#endif
  T2CONbits.TON = 1;                  // start timer per PWM
  
  // (TIMER 1 INITIALIZATION (TIMER IS USED AS A TRIGGER SOURCE FOR ALL CHANNELS).)
  
  T3CON=0;
  T3CONbits.TCS = 0;                  // clock from peripheral clock
//  T3CONbits.TCKPS = 2;                // 1:4 prescaler
//  PR3 = 1562;                         // rollover every n clocks; 1562= 16KHz per generare 1KHz di test
  T3CONbits.TCKPS = 1;                // 1:2 prescaler
  PR3 = PERIPHERAL_CLOCK_HZ/2/10000;    // 10KHz bandwidth
  T3CONbits.TON = 1;                  // start timer (usato per sinewave di test su DAC/PWM))

  IPC3bits.T3IP=4;            // set IPL 4, sub-priority 2??
  IPC3bits.T3IS=0;
  IEC0bits.T3IE=1;             // enable Timer 3 interrupt se si vuole

	}

void ADC_Init(void) {
  // https://www.microchip.com/forums/m878246-p2.aspx
  // o anche https://forum.mikroe.com/viewtopic.php?f=164&t=69452

  // PB3DIV=0x800a;    // boh divido per 10 NO! sono già attivi di default e impostati a 1/2 = 100MHz
  // e cmq ci vuole una sequenza di sblocco...
  // Set Peripheral Bus 2 Clock to SYSCLK/DIV_4
  /*SYSKEY = 0x00000000; // Start unlock sequence
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;
  while(PB2DIVbits.PBDIVRDY == 0)  ;
  PB2DIVbits.PBDIV = GetSystemClock() / GetPeripheral2Clock() - 1;
  PB2DIVbits.ON = 1;
  SYSKEY = 0x33333333;*/
 
  ADCCON1=0;    // AICPMPEN=0, siamo sopra 2.5V
  CFGCONbits.IOANCPEN=0;    // idem
  ADCCON2=0;
  ADCCON3=0;
  
    //Configure Analog Ports
    ADCCON3bits.VREFSEL = 0; //Set Vref to VREF+/-

#ifndef SSD1963
    ANSELBbits.ANSB4 = 1;
#else
    ANSELEbits.ANSE4 = 1;
#endif

    ADCCMPEN1=0x00000000;
    ADCCMPEN2=0x00000000;
    ADCCMPEN3=0x00000000;
    ADCCMPEN4=0x00000000;
    ADCCMPEN5=0x00000000;
    ADCCMPEN6=0x00000000;
    ADCFLTR1=0x00000000;
    ADCFLTR2=0x00000000;
    ADCFLTR3=0x00000000;
    ADCFLTR4=0x00000000;
    ADCFLTR5=0x00000000;
    ADCFLTR6=0x00000000;
    
    ADCTRGMODE=0;
    
    ADCTRG1=0;
    ADCTRG2=0;
    ADCTRG3=0;
#ifndef TEST_SINE_WAVE
#ifndef SSD1963
    ADCTRG2bits.TRGSRC4=5;      // Timer1
#else
    ADCCON1bits.STRGSRC=5;      // Timer1
#endif
#endif
    
#ifndef SSD1963
    ADCIMCON1bits.DIFF4 = 0; // single ended, signed
    ADCIMCON1bits.SIGN4 = 1; // 
#else
    ADCIMCON2bits.DIFF18 = 0; // single ended, signed
    ADCIMCON2bits.SIGN18 = 1; // 
#endif
   
  /* Initialize warm up time register */
  ADCANCON = 0;
  ADCANCONbits.WKUPCLKCNT = 5; // Wakeup exponent = 32 * TADx

    ADCEIEN1 = 0;
    
//    ADCCON2bits.ADCDIV = 64; // per SHARED: 2 TQ * (ADCDIV<6:0>) = 6 * TQ = TAD
    
    ADCCON3bits.ADCSEL = 0; //0=periph clock 3; 1=SYSCLK
    ADCCON3bits.CONCLKDIV = 0; // 100MHz, sotto è poi diviso 2 per il canale, = max 50MHz come da doc
    
    
#ifndef SSD1963
    ADCCON3bits.DIGEN4 = 1;
    ADCCON3bits.ADINSEL = 4;
#else
    ADCCON3bits.DIGEN7 = 1;
    ADCCON3bits.ADINSEL = 18;
#endif
    
#ifndef SSD1963
    ADC4TIMEbits.SELRES=0b01;        // 8 bits
    ADC4TIMEbits.ADCDIV=1;       // 
    ADC4TIMEbits.SAMC=4;        // 
#else
    ADCCON1bits.SELRES=0b01;
    ADCCON2bits.ADCDIV=1;
    ADCCON2bits.SAMC=3;
    ADCCON1bits.FRACT=1;
#endif
    
    ADCCSS1 = 0; // Clear all bits
    ADCCSS2 = 0;
#ifndef SSD1963
    ADCCSS1bits.CSS4 = 1;
#else
    ADCCSS1bits.CSS18 = 1;
#endif
    
    ADC0CFG=0;
#ifndef SSD1963
    ADC4CFG=DEVADC4;
#else
    ADC7CFG=DEVADC7;
#endif
    
    ADCCON1bits.ON = 1;   //Enable AD
    
  /* Wait for voltage reference to be stable */
#ifndef USING_SIMULATOR
  while(!ADCCON2bits.BGVRRDY); // Wait until the reference voltage is ready
  //while(ADCCON2bits.REFFLT); // Wait if there is a fault with the reference voltage
#endif

  // Enable clock to the module.
#ifndef SSD1963
  ADCCON3bits.DIGEN4 = 1; // Enable ADC4
  ADCANCONbits.ANEN4 = 1; // Enable clock, ADC4
#ifndef USING_SIMULATOR
  while(!ADCANCONbits.WKRDY4); // Wait until ADC4 is ready
#endif  
  ADCGIRQEN1bits.AGIEN4=1;
#else
  ADCCON3bits.DIGEN7 = 1; // Enable ADC7
  ADCANCONbits.ANEN7 = 1; // Enable clock, ADC7
#ifndef USING_SIMULATOR
  while(!ADCANCONbits.WKRDY7); // Wait until ADC7 is ready
#endif  
  ADCGIRQEN1bits.AGIEN18=1;
#endif

    //Turn on the Timer Interrupt
//    asm volatile("ei");
  T1CON=0;
  T1CONbits.TCS = 0;            // clock from peripheral clock
  T1CONbits.TCKPS = 0;          // 1:1 prescaler (DIVERSO dagli altri timer!))
//  PR1=100;        // ~25mS 24.7.19, con 10000 e PRE=256 e timer_clock=periph1_clock=100MHz, farebbe 39.6Hz=25mS
                  // 25.7.19, con 100 e PRE=64 e timer_clock=periph1_clock=100MHz, => 16KHz=64uS
                  // 25.7.19, con 50 e PRE=1 e timer_clock=periph1_clock=100MHz, => 2MHz
//  PR1=5000;     // test per audio, campiono a 20KHz!
//  con 50 pare fare 2Msamples, SAMC=3 e 8bits
#ifndef TEST_SINE_WAVE
  PR1 = PERIPHERAL_CLOCK_HZ/SAMPLING_RATE;
#else
  PR1 = (PERIPHERAL_CLOCK_HZ/SAMPLING_RATE) /*/1.5*/;      // in Test è lento... e i valori son tutti in vacca £$%&
  // v. anche DMAPRI, impostandolo sempra andare MA PORCAMADONNA RADDOPPIA l'ADC diobastardo
#endif

 
  T1CONbits.TON = 1;    // start timer to generate ADC triggers
//            ADCCON3bits.GSWTRG = 1;
    
      
	}

void DMA_Init(void) {
//  const unsigned int* sourceAddr = (void*) ((unsigned int) &ADCDATA4 & 0x1FFFFFFF);
  // usare VirtToPhys() ?? ma non lo trova, ecco KVA_TO_PA(
//  const unsigned int* sourceAddr = KVA_TO_PA(&ADCDATA4);
//https://www.microchip.com/forums/m951559.aspx

#ifndef SSD1963
  IEC6CLR = _IEC6_ADC4EIE_MASK; //Make sure ADC interrupt is disabled
  IFS6CLR = _IFS6_ADC4EIF_MASK;
#else
  IEC6CLR = _IEC6_ADC7EIE_MASK; //Make sure ADC interrupt is disabled
  IFS6CLR = _IFS6_ADC7EIF_MASK;
#endif
  
  IFS4CLR=_IFS4_DMA0IF_MASK; //clear any pending DMA channel 0 interrupt  
  
  DCH0CON = 0x0013;   // channel off, priority 3, no chaining, continous
  DCH0ECON = 0;    // no start or stop IRQs, no pattern match
#ifdef TEST_SINE_WAVE
  DCH0SSA = KVA_TO_PA(&testSine);  // transfer source physical address
  DCH0SSIZ = sizeof(testSine);                   // source size 4 bytes (2 "veri" + 2 finti per complex!))
#else
#ifndef SSD1963
  DCH0SSA = KVA_TO_PA(&ADCDATA4);  // transfer source physical address
#else
  DCH0SSA = KVA_TO_PA(((char*)&ADCDATA18)+2);  // transfer source physical address TRUCCHETTO per mettere 16bit ADC in Real e 0 in Imag!
#endif
  DCH0SSIZ = 4;                   // source size 4 bytes (2 "veri" + 2 finti per complex!))
#endif
  DCH0DSA = KVA_TO_PA(&srcCV1);   // transfer destination physical address
  DCH0DSIZ = sizeof(srcCV1);    // destination size 
  DCH0CSIZ = 4;                   // 4 bytes transferred per event
  DCH0INT = 0;                    // clear all interrupts
  //DCH0ECONSET = _ADC_DATA4_VECTOR << _DCH0ECON_CHSIRQ_POSITION;
  //DCH0ECONSET = _DCH0ECON_SIRQEN_MASK;
#ifdef TEST_SINE_WAVE
  DCH0ECONbits.CHSIRQ = _TIMER_1_VECTOR;
#else
#ifndef SSD1963
  DCH0ECONbits.CHSIRQ = _ADC_DATA4_VECTOR;
#else
  DCH0ECONbits.CHSIRQ = _ADC_DATA18_VECTOR;
#endif
#endif
  DCH0ECONbits.SIRQEN = 1;  // enable DMA 0 for IRQ trigger
  
  //DCH0CONbits.CHCHNS=1;        // Channel 0 chained to start from channel with lower natural priority (channel 1)
  //DCH0CONbits.CHCHN=1;        // enable chaining     
  
  DCH0INTbits.CHDDIE = 1;  // Interrupt when the fill is done.
  
  IPC33bits.DMA0IP=5;            // set IPL 5, sub-priority 2??
  IPC33bits.DMA0IS=0;
//  IPC33SET = 5 << _IPC33_DMA0IP_POSITION; // 
//  IPC33SET = 0 << _IPC33_DMA0IS_POSITION; // 
  IEC4bits.DMA0IE=1;             // enable DMA channel 0 interrupt
  //IEC4SET = _IEC4_DMA0IE_MASK; // enable DMA channel 0 interrupts

  DCH0CONbits.CHEN = 1; // turn on DMA channel 0
  DMACONbits.ON = 1;  // enable global DMA controller
    
  // https://www.microchip.com/forums/m698095.aspx
  // per chaining / pingpong..
  }

#if defined(ST7735) || defined(SSD1309)
extern UINT8 __attribute__((coherent)) __attribute__((aligned(16))) oled_buffer[SSD1309_LCDHEIGHT * SSD1309_LCDWIDTH / 8];
void DMA_Init2(void) {    // per SPI -> display

  IEC4CLR = _IEC4_SPI2TXIE_MASK; //Make sure SPI TX interrupt is disabled
  IFS4CLR = _IFS4_SPI2TXIF_MASK;
  
  IFS4CLR=_IFS4_DMA1IF_MASK; //clear any pending DMA channel 1 interrupt
  
  DCH1CON = 0x0002;   // channel off, priority 2, no chaining, one-shot
  DCH1ECON = 0;    // no start or stop IRQs, no pattern match
  DCH1SSA = KVA_TO_PA(&oled_buffer);  // transfer source physical address
  DCH1SSIZ = sizeof(oled_buffer);                   // source size 
  DCH1DSA = KVA_TO_PA(&SPI2BUF);   // transfer destination physical address
  DCH1DSIZ = 1 /*4*/;    // destination size 
  DCH1CSIZ = 1;                   // 1 bytes transferred per event
  DCH1INT = 0;                    // clear all interrupts
  //DCH0ECONSET = _ADC_DATA4_VECTOR << _DCH0ECON_CHSIRQ_POSITION;
  //DCH0ECONSET = _DCH0ECON_SIRQEN_MASK;
  DCH1ECONbits.CHSIRQ = _SPI2_TX_VECTOR;
  DCH1ECONbits.SIRQEN = 1;  // enable DMA 1 for IRQ trigger
  
  DCH1INTbits.CHBCIE /*CHDDIE*/ = 1;  // Interrupt when block transfer complete [the fill is done].

  m_SPICSBit=0;   // un pizzico prima di far partire la cosa!
  
  
  DMACONbits.ON = 1;  // enable global DMA controller  C'E' già!
  
  IPC33bits.DMA1IP=2;            // set IPL 2, sub-priority 2??
  IPC33bits.DMA1IS=0;
//  IPC33SET = 2 << _IPC33_DMA1IP_POSITION; // 
//  IPC33SET = 0 << _IPC33_DMA1IS_POSITION; // 
  IEC4bits.DMA1IE=1;             // enable DMA channel 1 interrupt
  //IEC4SET = _IEC4_DMA1IE_MASK; // enable DMA channel 1 interrupts
  
  DCH1CONbits.CHEN = 1; // turn on DMA channel 1  

  }
#endif

void PWM_Init(void) {

  CFGCONbits.OCACLK=0;      // sceglie timer per PWM
  
#ifdef SSD1963
  OC3CON = 0x0006;      // TimerX ossia Timer2; PWM mode no fault; Timer 16bit, TimerX
  OC3R    = 1000;		 // su PIC32 è read-only!
  OC3RS   = 500;   // 50%, relativo a PR2 del Timer2
  OC3CONbits.ON = 1;   // on
#else
  OC1CON = 0x0006;      // TimerX ossia Timer2; PWM mode no fault; Timer 16bit, TimerX
  OC1R    = 500;		 // su PIC32 è read-only!
  OC1RS   = 1000;   // 50%, relativo a PR2 del Timer2
  OC1CONbits.ON = 1;   // on
#endif

  }

#ifdef SSD1963
void SPI_Init(void) {

	SPISDITris=1;				// SDI è input
	SPISDOTris=0;				// SDO è output
	SPISCKTris=0;				// SCK è output
	SPICSTris=0;
  CNPUDbits.CNPUD2=1;     // serve sempre!
  m_SPICSBit=1;

  SPI1CON=  0b00000000000000000000000100100000;    // master, mode 0
  SPI1CON2= 0b00000000000000000000000000000000;    // no special length; 
  SPI1STAT= 0b00000000000000000000000000000000;    // 
  SPI1BRG=19 /* (GetPeripheralClock() / freq /2) -1 */;    // 2.55MHz 19/8/22
  // il datasheet più o meno indica 2.5MHz max, come pure le librerie in giro
  
  SPI1CONbits.ON=1;
  }

uint8_t spiXFer(uint8_t c) {
  uint8_t  b;
  
  SPI1STATbits.SPIROV = 0;  // Reset overflow bit
  SPI1BUF = c;     // Write character to SPI buffer
  while(SPI1STATbits.SPIBUSY == 1 /*(SPI1STATbits.SPITBF == 1)*/ /* || (SPI1STATbits.SPIRBF == 0) */ )
  	ClrWdt();
  b=SPI1BUF;
//  while(SPI1STATbits.SPITBF)   // Wait until transmission is started
//    ;
  return b;
  }

#define Z_THRESHOLD     300
#define Z_THRESHOLD_INT	75
#define MSEC_THRESHOLD  3
static int16_t besttwoavg(int16_t x, int16_t y, int16_t z) {
  int16_t da, db, dc;
  int16_t reta = 0;
  
  if(x > y) 
    da = x - y; 
  else 
    da = y - x;
  if(x > z) 
    db = x - z; 
  else 
    db = z - x;
  if(z > y)
    dc = z - y; 
  else 
    dc = y - z;

  if(da <= db && da <= dc ) 
    reta = (x + y) >> 1;
  else if(db <= da && db <= dc)
    reta = (x + z) >> 1;
  else reta = (y + z) >> 1;   //    else if ( dc <= da && dc <= db ) reta = (x + y) >> 1;

  return reta;
  }
//https://github.com/PaulStoffregen/XPT2046_Touchscreen/blob/master/XPT2046_Touchscreen.cpp
//https://github.com/spapadim/XPT2046/blob/master/XPT2046.cpp
  BYTE readTouchScreen(uint16_t *xr, uint16_t *yr, uint8_t *zr) {
  uint8_t data[6];
  int z;
  
  SPI_CS_LOW();
  SPI1CONbits.MODE16=0;
  spiXFer(0xB1 /* Z1 */);
  SPI1CONbits.MODE16=1;
  int16_t z1 = spiXFer(0xC1 /* Z2 */) >> 3;
  z = z1 + 4095;
  int16_t z2 = spiXFer(0x91 /* X */) >> 3;
  z -= z2;
  if(z >= Z_THRESHOLD) {
    spiXFer(0x91 /* X */);  // dummy X measure, 1st is always noisy
    data[0] = spiXFer(0xD1 /* Y */) >> 3;
    data[1] = spiXFer(0x91 /* X */) >> 3; // make 3 x-y measurements
    data[2] = spiXFer(0xD1 /* Y */) >> 3;
    data[3] = spiXFer(0x91 /* X */) >> 3;
    }
  else 
    data[0] = data[1] = data[2] = data[3] = 0;	// Compiler warns these values may be used unset on early exit.
  data[4] = spiXFer(0xD0 /* Y */) >> 3;	// Last Y touch power down
  data[5] = spiXFer(0) >> 3;
  SPI1CONbits.MODE16=0;
  SPI_CS_HIGH();
  
  if(z < 0) 
    z = 0;
	if(z < Z_THRESHOLD) { //	if ( !touched ) {
		*zr = 0;
		if(z < Z_THRESHOLD_INT) { //	if ( !touched ) {
//			if(255 != tirqPin) isrWake = false;
    	}
		return;
  	}
	*zr = z >> 4;
  
  *xr = besttwoavg( data[0], data[2], data[4] );
	*yr = besttwoavg( data[1], data[3], data[5] );
  
  if(z >= Z_THRESHOLD) {
//		msraw = now;	// good read completed, set wait
    return 1;
    }
  return 0;
  }

#endif

void UART_Init(DWORD baudRate) {
  
#ifdef PCB2019
  U6MODE=0b0000000000001000;    // BRGH=1
  U6STA= 0b0000010000000000;    // TXEN
  DWORD baudRateDivider = ((GetPeripheralClock()/(4*baudRate))-1);
  U6BRG=baudRateDivider;
  U6MODEbits.ON=1;
#else
  U1MODE=0b0000000000001000;    // BRGH=1
  U1STA= 0b0000010000000000;    // TXEN
  DWORD baudRateDivider = ((GetPeripheralClock()/(4*baudRate))-1);
  U1BRG=baudRateDivider;
  U1MODEbits.ON=1;
#endif
  
#if 0
  ANSELDCLR = 0xFFFF;
  CFGCONbits.IOLOCK = 0;      // PPS Unlock
  RPD11Rbits.RPD11R = 3;        // Assign RPD11 as U1TX
  U1RXRbits.U1RXR = 3;      // Assign RPD10 as U1RX
  CFGCONbits.IOLOCK = 1;      // PPS Lock

  // Baud related stuffs.
  U1MODEbits.BRGH = 1;      // Setup High baud rates.
  unsigned long int baudRateDivider = ((GetSystemClock()/(4*baudRate))-1);
  U1BRG = baudRateDivider;  // set BRG

  // UART Configuration
  U1MODEbits.ON = 1;    // UART1 module is Enabled
  U1STAbits.UTXEN = 1;  // TX is enabled
  U1STAbits.URXEN = 1;  // RX is enabled

  // UART Rx interrupt configuration.
  IFS1bits.U1RXIF = 0;  // Clear the interrupt flag
  IFS1bits.U1TXIF = 0;  // Clear the interrupt flag

  INTCONbits.MVEC = 1;  // Multi vector interrupts.

  IEC1bits.U1RXIE = 1;  // Rx interrupt enable
  IEC1bits.U1EIE = 1;
  IPC7bits.U1IP = 7;    // Rx Interrupt priority level
  IPC7bits.U1IS = 3;    // Rx Interrupt sub priority level
#endif
  }

char BusyUART1(void) {
  
#ifdef PCB2019
  return(!U6STAbits.TRMT);
#else
  return(!U1STAbits.TRMT);
#endif
  }

void putsUART1(unsigned int *buffer) {
  char *temp_ptr = (char *)buffer;

    // transmit till NULL character is encountered 

  if(U1MODEbits.PDSEL == 3)        /* check if TX is 8bits or 9bits */
    {
        while(*buffer) {
#ifdef PCB2019
            while(U6STAbits.UTXBF); /* wait if the buffer is full */
            U6TXREG = *buffer++;    /* transfer data word to TX reg */
#else
            while(U1STAbits.UTXBF); /* wait if the buffer is full */
            U1TXREG = *buffer++;    /* transfer data word to TX reg */
#endif
        }
    }
  else {
        while(*temp_ptr) {
#ifdef PCB2019
            while(U6STAbits.UTXBF);  /* wait if the buffer is full */
            U6TXREG = *temp_ptr++;   /* transfer data byte to TX reg */
#else
            while(U1STAbits.UTXBF);  /* wait if the buffer is full */
            U1TXREG = *temp_ptr++;   /* transfer data byte to TX reg */
#endif
        }
    }
  }

unsigned int ReadUART1(void) {
  
#ifdef PCB2019
  if(U6MODEbits.PDSEL == 3)
    return (U6RXREG);
  else
    return (U6RXREG & 0xFF);
#else
  if(U1MODEbits.PDSEL == 3)
    return (U1RXREG);
  else
    return (U1RXREG & 0xFF);
#endif
  }

void WriteUART1(unsigned int data) {
  
#ifdef PCB2019
  if(U6MODEbits.PDSEL == 3)
    U6TXREG = data;
  else
    U6TXREG = data & 0xFF;
#else
  if(U1MODEbits.PDSEL == 3)
    U1TXREG = data;
  else
    U1TXREG = data & 0xFF;
#endif
  }


// GP Math Routines
// Converts int32c to float magnitude (Exec. time about 400nS on 200 MHz PIC32MZ)
float Complex32ToMag(int32c a) {
  float mag;
  float tempRe;
  float tempIm;

  tempRe = (double)a.re * (double)a.re;
  tempIm = (double)a.im * (double)a.im;
  mag = sqrt(tempRe + tempIm);
  return mag;           
  }




// ---------------------------------------------------------------------
#if defined(ST7735) || defined(SSD1309) || defined(SSD1963)
BYTE LCD_init(void) {		// su SPI

#if defined(ST7735)
  __delay_ms(1000);     // SERVE condensatore su RST pin...

//	TFT_ILI9163C();
  Adafruit_ST7735_1(0,0,0,0,-1);
  Adafruit_ST7735_initR(INITR_BLACKTAB);
  
  displayInit(NULL);
#elif defined(SSD1309)
  __delay_ms(1000);     // SERVE condensatore su RST pin...
#ifndef USA_SPI_HW
	Adafruit_SSD1309_SWSPI(0,1,2,3,13 /*RB13*/);
#else
	Adafruit_SSD1309_HWSPI(0,1,2 /*RB13*/);
#endif
	begin(SSD1309_SWITCHCAPVCC, SSD1309_I2C_ADDRESS, 0);
  
	setTextSize(1);
	setTextColor(1);
	setTextColorBG(1, 0);
	setTextWrap(1);
  
#elif defined(SSD1963)
  
  __delay_ms(500);     // reset on board

//  SSD1963_PORTB_SSD1963_PORTB(TFTBUS24);
//  SSD1963_PORTB_initInterface();    // QUALE DELLE 2??
  SSD1963_initializeDevice2(TFT7);
  SSD1963_enableBacklight();
  
#endif
  
#ifdef m_LCDBLBit
    m_LCDBLBit=1;
#endif
  
#if defined(ST7735) || defined(SSD1309)
  display();    //	LCD_cls(); così mostra il logo!
#elif defined(SSD1963)
  LCD_cls();
#endif

// init done
	setTextWrap(1);
//	setTextColor2(WHITE, BLACK);

//	drawBG();
  
  __delay_ms(500);
  ClrWdt();
#if defined(ST7735) || defined(SSD1309)
  clearDisplay();
#elif defined(SSD1963)
  fillScreen(BLACK);
#endif

	return 1;
	}


void LCD_XY(BYTE x, BYTE y) {
		
	setCursor(x*textsize*6,y*textsize*8);
	}
	

void LCD_WriteDec(BYTE n) {

// sbianchettare, anche?
	LCD_PutChar((n / 10) + '0');
	LCD_PutChar((n % 10) + '0');
	}

/* RIFARE CON FUNZIONI ILI
extern BYTE gfx_buffer1[];
void LCD_Scroll(void) {
	BYTE x,y,yMax;
	int chunk;

	yMax=8;

	chunk=128 // orientamento è 
	for(y=1; y<yMax; y++) {
		memcpy(gfx_buffer1+((y-1)*chunk),gfx_buffer1+(y*chunk),chunk);
		}
	memset(gfx_buffer1+((y-1)*chunk),0,chunk);

	}	
*/
#endif


// ---------------------------------------------------------------------
void loadSettings(void) {
	BYTE i;
	
  /* mettere!
  EEinit();
  { 
    BYTE *p=(BYTE*)&configParms;
    int n;
    for(n=0; n<sizeof(struct SAVED_PARAMETERS); n++)
      p[n]=EEread(n);
   }
  */
  if(configParms.signature != 0x4447) {
    configParms.signature=0x4447;
    configParms.waterfall=0;
    configParms.MW_LW=1;
    configParms.AM_FM_SSB=1;
    
    configParms.spectrum=1;
		}

	}

void saveSettings(void) {
	BYTE i;

//  AreaEEWrite(&configParms, sizeof(configParms));

	}


// ---------------------------------------------------------------------

//#warning USARE define (SINE_TABLE_7BIT) e    const q15_t sine_table[] = {0, 402, 804, 1206, 1607, 2009, 2410, 2811,
DWORD samplesPerSecond;
double frequency;
double amplitude;
double oscPhase;
double oscPhaseIncrement;
BYTE modulation;
double frequencyModulation;
BYTE ratioModulation;

void prepareLocalOsc(uint8_t n, double ampl) {
  int i,j;
  
  frequency=Fs;
  CCosOscillator_CCosOscillator(Fs, getLocOscFreqfromTune(n),ampl);
  CSinOscillator_CSinOscillator(Fs, getLocOscFreqfromTune(n),ampl);
  // qua precalcolo la tabella con tutti i campioni giusti, così non devo fare nextSample "al volo"
	oscPhase = 0;
	for(i=0; i<TABLE_LOCALOSC_SIZE; i++) {
		j = COscillator_nextSample_idx();
		local_oscillator_table[i].re = oscillator_cosTable[j];
		local_oscillator_table[i].im = oscillator_sinTable[j];
		}
  }

void COscillator_COscillator(DWORD samplesPerSec, int freq, double ampl) {

	oscPhase = 0;
	modulation=0;
  amplitude=ampl;

	samplesPerSecond = samplesPerSec;
	frequency = freq;
	oscPhaseIncrement = 2 * PI * frequency / (double)samplesPerSecond;

	frequencyModulation=0;
	}

void COscillator_COscillator_(void) {

	}

int COscillator_nextSample_idx(void) {

	oscPhase += oscPhaseIncrement;
	if(frequency > 0 && oscPhase >= 2 * PI)
		oscPhase = oscPhase - 2*PI;
	if(frequency < 0 && oscPhase <= 0)
		oscPhase = oscPhase + 2*PI;
	int idx = (int)(((int)(oscPhase * (double)TABLE_LOCALOSC_SIZE/(2 * PI))) % TABLE_LOCALOSC_SIZE);

	return idx;
	}

q15 COscillator_nextSample(void) {

	oscPhase += oscPhaseIncrement;
	if(frequency > 0 && oscPhase >= 2 * PI)
		oscPhase = oscPhase - 2*PI;
	if(frequency < 0 && oscPhase <= 0)
		oscPhase = oscPhase + 2*PI;
	int idx = (int)(((int)(oscPhase * (double)TABLE_LOCALOSC_SIZE/(2 * PI))) % TABLE_LOCALOSC_SIZE);

//	q15_t q15_sin(q16angle_t theta)

	return oscillator_sinTable[idx];
	}

void COscillator_setFrequency(double freq) {

	frequency = freq;
	oscPhaseIncrement = 2 * PI * frequency / (double)samplesPerSecond;		
	}

void COscillator_setModulation(double freq,BYTE mode,BYTE ratio) {

	modulation=mode;
	frequencyModulation=freq;
	ratioModulation=ratio;
	}


void CCosOscillator_CCosOscillator(DWORD samplesPerSec, int freq, double ampl) {
  int n;

  COscillator_COscillator(samplesPerSec, freq, ampl);
	samplesPerSecond=samplesPerSec; frequency=freq;		// non riesco a chiamare il costruttore base £$%@#
	oscPhaseIncrement = 2 * PI * frequency / (double)samplesPerSecond;		
	for(n=0; n<TABLE_LOCALOSC_SIZE; n++) {
		oscillator_cosTable[n] = Q15(cos(n*2.0*PI/(double)TABLE_LOCALOSC_SIZE) * amplitude);
		}
	}


void CSinOscillator_CSinOscillator(DWORD samplesPerSec, int freq, double ampl) {
  int n;

  COscillator_COscillator(samplesPerSec, freq, ampl);
	samplesPerSecond=samplesPerSec; frequency=freq;
	oscPhaseIncrement = 2 * PI * frequency / (double)samplesPerSecond;		
	for(n=0; n<TABLE_LOCALOSC_SIZE; n++) {
		oscillator_sinTable[n] = Q15(sin(n*2.0*PI/(double)TABLE_LOCALOSC_SIZE) * amplitude);
		}
	}


void Complex_multiply(int32c n1,int32c n2,int32c *ret) { 
  
  ret->re=(n1.re*n2.re)-(n1.im*n2.im);
		/* if (a,b)(c,d) then formula of multiplication is (ac-bd,ad+bc) */ 
	ret->im=(n1.re*n2.im)+(n1.im*n2.re); 
  }
// verificare se è più veloce passare puntatore a ret o restituire... ma visto che il valore va poi in array direi così
void /*fractcomplex32*/ Complex_Real_multiply(q15 n1,int16c n2,int16c *ret) { 
//  fractcomplex32 ret;
  ret->re=(n1*n2.re);
	ret->im=(n1*n2.im);
//  return ret;
  }

void initHamming(void) {
  int i;

  for(i=0; i<FFT_BLOCK_LENGTH; i++)     // init window (Hamming)
    hammingWindow[i] = Q15(0.54 - 0.46 * cosf(2.0*PI*(float)i /(float)(FFT_BLOCK_LENGTH-1))); 
  } 

q15 CFirFilter_coeffs32[] = { 
	Q15(-0.006907390651426909),
	Q15(-0.005312580651013577),
	Q15(-0.002644739091749330),
	 Q15(0.001183336967173867),
	 Q15(0.006192514984703184),
	 Q15(0.012327624024018900),
	 Q15(0.019453074574047193),
	 Q15(0.027354594681004485),
	 Q15(0.035747363433714395),
	 Q15(0.044290256550950494),
	 Q15(0.052605352818230700),
	 Q15(0.060301340430963822),
	 Q15(0.066999061029964030),
	 Q15(0.072357178973056519),
	 Q15(0.076095892581739766),
	 Q15(0.078016723675218447),
	 Q15(0.078016723675218447),
	 Q15(0.076095892581739766),
	 Q15(0.072357178973056519),
	 Q15(0.066999061029964030),
	 Q15(0.060301340430963822),
	 Q15(0.052605352818230700),
	 Q15(0.044290256550950494),
	 Q15(0.035747363433714395),
	 Q15(0.027354594681004485),
	 Q15(0.019453074574047193),
	 Q15(0.012327624024018900),
	 Q15(0.006192514984703184),
	 Q15(0.001183336967173867),
	Q15(-0.002644739091749330),
	Q15(-0.005312580651013577),
	Q15(-0.006907390651426909)
	};
q15 CFirFilter_coeffs[1];      // fare se si vuole...

q15 CFirFilter_gain = Q15(1);
//BYTE CFirFilter_M = sizeof(CFirFilter_coeffs5)/sizeof(q15)-1;
q15 CFirFilter_xv32[sizeof(CFirFilter_coeffs32)/sizeof(q15)];		//
q15 CFirFilter_xv[256];		//
  
q15 CFirFilter_filter6(q15 in) {
  
  // v. anche mips_fir16()
//e PARM_EQUAL_FILTER_16  
  
	q15 *f1,*f2; 
  int32_t sum;
	BYTE i;

#if 1
//	for(i=0; i < sizeof(CFirFilter_coeffs5)/sizeof(q15)-1; i++) 
//		CFirFilter_xv4[i] = CFirFilter_xv4[i+1];
  memcpy(&CFirFilter_xv4[0],&CFirFilter_xv4[1],sizeof(CFirFilter_coeffs5)-sizeof(q15));
//	CFirFilter_xv[CFirFilter_M] = /*libq_q31_Mult2_q15_q15*/q15_mul(in,CFirFilter_gain);
	CFirFilter_xv4[sizeof(CFirFilter_coeffs5)/sizeof(q15)-1] = in;
  f1=&CFirFilter_coeffs5[0];
  f2=&CFirFilter_xv4[0];
  sum = Q15_MUL(*f1++ , *f2++) ;  // /*libq_q31_Mult2_q15_q15*/q15_mul(*f1++,*f2++);
//  sum = libq_q31_Mult2_q15_q15(CFirFilter_coeffs5[i], CFirFilter_xv[i]);
	i = sizeof(CFirFilter_coeffs5)/sizeof(q15)-1; 
  while(i--) 
		sum += Q15_MUL(*f1++ , *f2++); ///*libq_q31_Mult2_q15_q15*/q15_mul(*f1++,*f2++);
#else
	for(i=0; i < sizeof(CFirFilter_coeffs5)/sizeof(q15)-1; i++) 
		CFirFilter_xv4[i] = CFirFilter_xv4[i+1];
	CFirFilter_xv4[4] = in;
	sum = 0;
	for(i=0; i <= 4; i++) 
		sum += (CFirFilter_coeffs5[i] * CFirFilter_xv4[i]);
#endif

	return sum;
	}

q15 CFirFilter_filter32(q15 in) {
  
  // v. anche mips_fir16()
//e PARM_EQUAL_FILTER_16  
  
	q15 sum; 
	BYTE i;

	for(i=0; i < sizeof(CFirFilter_coeffs32)/sizeof(q15)-1; i++) 
		CFirFilter_xv32[i] = CFirFilter_xv32[i+1];
#warning USARE MEMCPY!
//	CFirFilter_xv[CFirFilter_M] = /*libq_q31_Mult2_q15_q15*/q15_mul(in,CFirFilter_gain);
	CFirFilter_xv32[sizeof(CFirFilter_coeffs32)/sizeof(q15)-1] = in;
  sum = /*libq_q31_Mult2_q15_q15*/q15_mul(CFirFilter_coeffs32[0], CFirFilter_xv32[0]);
//  sum = libq_q31_Mult2_q15_q15(CFirFilter_coeffs5[i], CFirFilter_xv[i]);
	for(i=1; i < sizeof(CFirFilter_coeffs32)/sizeof(q15); i++) 
		sum += /*libq_q31_Mult2_q15_q15*/q15_mul(CFirFilter_coeffs32[i], CFirFilter_xv32[i]);

	return sum;
	}

q15 CFirFilter_filter(q15 in) {
  // GESTIRE se si vuole usare! per Poly sotto la metto inline, unrollata e fissata a 16 => 256
	q15 sum; 
	BYTE i;

	for(i=0; i < sizeof(CFirFilter_coeffs)/sizeof(q15)-1; i++) 
		CFirFilter_xv[i] = CFirFilter_xv[i+1];
//	CFirFilter_xv[CFirFilter_M] = /*libq_q31_Mult2_q15_q15*/q15_mul(in,CFirFilter_gain);
	CFirFilter_xv32[sizeof(CFirFilter_coeffs)/sizeof(q15)-1] = in;
  sum = /*libq_q31_Mult2_q15_q15*/q15_mul(CFirFilter_coeffs[0], CFirFilter_xv[0]);
//  sum = libq_q31_Mult2_q15_q15(CFirFilter_coeffs5[i], CFirFilter_xv[i]);
	for(i=1; i < sizeof(CFirFilter_coeffs32)/sizeof(q15); i++) 
		sum += /*libq_q31_Mult2_q15_q15*/q15_mul(CFirFilter_coeffs[i], CFirFilter_xv[i]);

	return sum;
	}


#define	NUM_SUBFILTERS 16
q15 CPolyPhaseFilter_coeffs[4*NUM_SUBFILTERS];
q15 CPolyPhaseFilter_taps[4][NUM_SUBFILTERS];
q15 *CFirFilter_initRaiseCosine(q15 *coeffs, double sampleRate, double freq, double alpha, DWORD len) {
	int M = len-1;
	double Fc = freq/sampleRate;
	int i;
	
	double sumofsquares = 0;
	double tempCoeffs[len];
	int limit = (int)(0.5 / (alpha * Fc));
	for(i=0; i <= M; i++) {
		double sinc = (sin(2 * PI * Fc * (i - M/2)))/ (i - M/2);
		double cosc = cos(alpha * PI * Fc * (i - M/2)) / ( 1 - (pow((2 * alpha * Fc * (i - M/2)),2)));
		
		if(i == M/2) {
			tempCoeffs[i] = 2 * PI * Fc * cosc;
			} 
		else {
			tempCoeffs[i] = sinc * cosc;
			}
		
		// Care because ( 1 - ( 2 * pow((alpha * Fc * (i - M/2)),2))) is zero for 
		if((i-M/2) == limit || (i-M/2) == -limit) {
			tempCoeffs[i] = 0.25 * PI * sinc;
			} 
		
		sumofsquares += tempCoeffs[i]*tempCoeffs[i];
		}
	double maxval = sqrt(sumofsquares);		// 0.08 con 4tap, 0.23 con 32, 0.30 con 64, 0.34 con 128
	maxval *= log(M)/log(2);					// 0.13 @4, 2.39 @128
	for(i=0; i < len; i++) {
		coeffs[i] = Q15(tempCoeffs[len-i-1]/maxval);
		}
	return coeffs;
	}

void CPolyPhaseFilter_initSubFilters(q15 *coeffs, WORD numValues, BYTE R) {
	int M = numValues;
//	NUM_SUBFILTERS=R;
//	subFilters = new CFirFilter*[numSubfilters];
	int P = R - 1; // position of the polyphase switch
	int i,j;

	for(j=0; j < R; j++) {		//0..16
		for(i=0; i < M/R; i++) {		//0..4
			CPolyPhaseFilter_taps[j][i] = coeffs[P + i*R]; 
			}
//		subFilters[j] = new CFirFilter(taps,i);
		P--;
		if(P < 0)
			P = R - 1;
		}
	}

void CPolyPhaseFilter_CPolyPhaseFilter(DWORD sampleRate, DWORD freq, DWORD decimationRate, DWORD len, double gain) {
  
  CFirFilter_initRaiseCosine(CPolyPhaseFilter_coeffs, sampleRate, freq, 0.5 /*alpha*/, len);
  CPolyPhaseFilter_initSubFilters(CPolyPhaseFilter_coeffs, len, decimationRate);
	}
  
q15 CPolyPhaseFilter_filter(q15 *in,DWORD numValues) {
//	q15 sum; 
  int32_t sum;
	int j = numValues-1;
	DWORD i;
  q15 *p1,*p2;

	sum = 0;
	for(i=0; i < numValues; i++) {
//		sum += subFilters[i]->filter(in[j--]);

    p2=&CFirFilter_xv[i*4];
    p1=CPolyPhaseFilter_taps[i];

    memcpy(p2,p2+1,(4-1)*sizeof(q15));
    CFirFilter_xv[i*4+4-1] = in[j--];
#if 0
    sum += Q15_MUL(CPolyPhaseFilter_taps[i][0], CFirFilter_xv[i*4+0]);  // /*libq_q31_Mult2_q15_q15*/q15_mul(*f1++,*f2++);
    sum += Q15_MUL(CPolyPhaseFilter_taps[i][1], CFirFilter_xv[i*4+1]);
    sum += Q15_MUL(CPolyPhaseFilter_taps[i][2], CFirFilter_xv[i*4+2]);
    sum += Q15_MUL(CPolyPhaseFilter_taps[i][3], CFirFilter_xv[i*4+3]);
#else
    sum += Q15_MUL(*p1++, *p2++);  // /*libq_q31_Mult2_q15_q15*/q15_mul(*f1++,*f2++);
    sum += Q15_MUL(*p1++, *p2++);
    sum += Q15_MUL(*p1++, *p2++);
    sum += Q15_MUL(*p1, *p2);
#endif
    // risparimo almeno 20uS usando puntatori
    }
	return sum /* *m_gain */;
	}
	

// ****************************************************************************
uint8_t drawJPEG(UGRAPH_COORD_T left,UGRAPH_COORD_T top,UGRAPH_COORD_T right,UGRAPH_COORD_T bottom,
        const uint8_t *image,uint32_t imagesize) {
  pjpeg_image_info_t JPG_Info;
  int mcu_x=0;
  int mcu_y=0;
  int status;
  char buffer[32];
  UGRAPH_COORD_T x,y,x1,y1;

  jpegBufferPtr=(uint8_t *)image;
  jpegBufferLen=imagesize;

  if(!pjpeg_decode_init(&JPG_Info, pjpeg_need_bytes_callback, NULL,0)) {
//              DrawRectangle(img_ofs_x,img_ofs_y,img_ofs_x+appData.videoStreamFormat.videoSize.cx+1,
//                      img_ofs_y+appData.videoStreamFormat.videoSize.cy+1,LIGHTGREEN);

/*    setTextColor(BRIGHTRED);
LCD_XY(4,5);
sprintf(buffer,"JPEG: %u*%u",JPG_Info.m_width,JPG_Info.m_height);
LCD_Write(buffer);
__delay_ms(1000);*/


    for(;;) {

      if(status = pjpeg_decode_mcu())
        goto error_compressed;

      for(y=0; y < JPG_Info.m_MCUHeight; y += 8) {
        y1=(mcu_y*JPG_Info.m_MCUHeight) + y;
        for(x=0; x < JPG_Info.m_MCUWidth; x += 8) {
          x1=(mcu_x*JPG_Info.m_MCUWidth) + x  /* * JPG_Info.m_comps*/;

          // Compute source byte offset of the block in the decoder's MCU buffer.
          uint32_t src_ofs = (x * 8U) + (y * 16U);
          const uint8_t *pSrcR = JPG_Info.m_pMCUBufR + src_ofs;
          const uint8_t *pSrcG = JPG_Info.m_pMCUBufG + src_ofs;
          const uint8_t *pSrcB = JPG_Info.m_pMCUBufB + src_ofs;

			    setAddrWindow(x1+left,y1+top,8,8);
          m_LCDRSBit = 1;
				  m_LCDCSBit = 0;

          uint8_t bx,by;
          for(by=0; by<8; by++) {
            for(bx=0; bx<8; bx++) {
//              SSD1963_setPixel(x1+bx, y1+by, Color565(*pSrcR,*pSrcG,*pSrcB));
              SSD1963_data_raw(Color565(*pSrcR,*pSrcG,*pSrcB));
              pSrcR++; pSrcG++; pSrcB++;
              }
            }
				  m_LCDCSBit = 1;
          }
//                  x1+=JPG_Info.m_MCUWidth;
        }
//                y1+=JPG_Info.m_MCUHeight;

      mcu_x++;      // in x ogni blocco è già 16 pixel (con YUV, pare)
      if(mcu_x >= JPG_Info.m_MCUSPerRow) {
        mcu_x=0;
        mcu_y++;
//                  if(mcu_y == JPG_Info.m_MCUSPerCol) {
//                    break;
//                   }
        }
      ClrWdt();
      }
    }
  
  return 1;
  
error_compressed:
  return 0; 
  }

uint8_t readArea(UGRAPH_COORD_T left,UGRAPH_COORD_T top,
        UGRAPH_COORD_T right,UGRAPH_COORD_T bottom,uint16_t *area) {
  UGRAPH_COORD_T x,y;
/*  char buffer[64];
    setTextColor(BRIGHTRED);
LCD_XY(4,20);
sprintf(buffer,"get: %u,%u,%u,%u",left,top,right,bottom);
LCD_Write(buffer);*/

  SSD1963_command(SSD1963_SetColumnAddress);
  SSD1963_data8(HIBYTE(left));
  SSD1963_data8(LOBYTE(left));
  SSD1963_data8(HIBYTE(right));
  SSD1963_data8(LOBYTE(right));
  SSD1963_command(SSD1963_SetPageAddress);
  SSD1963_data8(HIBYTE(top));
  SSD1963_data8(LOBYTE(top));
  SSD1963_data8(HIBYTE(bottom));
  SSD1963_data8(LOBYTE(bottom));
  SSD1963_command(SSD1963_ReadMemoryStart);
  
  for(y=top; y<=bottom; y++) {
    for(x=left; x<=right; x++) {
      *area++=SSD1963_read();
      }
    }
  return 1;
  }

uint8_t putArea(UGRAPH_COORD_T left,UGRAPH_COORD_T top,
        UGRAPH_COORD_T right,UGRAPH_COORD_T bottom,const uint16_t *area) {
  UGRAPH_COORD_T x,y;
  
//  			    setAddrWindow(x1+left,y1+top,8,8);
/*    fillRect(30,155,250,100,BLACK);
    setTextColor(BRIGHTRED);
LCD_XY(4,22);
sprintf(buffer,"put: %u,%u,%u,%u, %02X %02X %02X %02X",left,top,right,bottom,
        area[0],area[1],area[2],area[3]);
LCD_Write(buffer);*/

  SSD1963_command(SSD1963_SetColumnAddress);
  SSD1963_data8(HIBYTE(left));
  SSD1963_data8(LOBYTE(left));
  SSD1963_data8(HIBYTE(right));
  SSD1963_data8(LOBYTE(right));
  SSD1963_command(SSD1963_SetPageAddress);
  SSD1963_data8(HIBYTE(top));
  SSD1963_data8(LOBYTE(top));
  SSD1963_data8(HIBYTE(bottom));
  SSD1963_data8(LOBYTE(bottom));
  SSD1963_command(SSD1963_WriteMemoryStart);
  
  m_LCDRSBit = 1;
  m_LCDCSBit = 0;
  for(y=top; y<=bottom; y++) {
    for(x=left; x<=right; x++) {
      SSD1963_data_raw(*area++);
      }
    }
  m_LCDCSBit = 1;
  return 1;
  }

void drawAgo(int x) {
  
  drawFastVLine(x-2,5,169,COLORE_AGO1);
  drawFastVLine(x-1,5,170,COLORE_AGO);
  drawFastVLine(x,5,170,COLORE_AGO);
  drawFastVLine(x+1,5,170,COLORE_AGO);
  drawFastVLine(x+2,5,169,COLORE_AGO2);
  }



