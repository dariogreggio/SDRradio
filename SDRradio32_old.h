/* 
 * File:   SDRradio32.h
 * Author: dario
 *
 * Created on 13 settembre 2017, 23.53
 */

#ifndef SDRRADIO32_H
#define	SDRRADIO32_H

#include <xc.h>
#include "libmathq15.h"
#include "../../src_mz/libq_c.h"

#ifdef	__cplusplus
extern "C" {
#endif

//#define NO_LCD_DEBUG 1        // perché i pin di PORTB (su SSD1963) sono usati dal Debugger... e si pianta
    
/* check if build is for a real debug tool */
#if defined(__DEBUG) && !defined(__MPLAB_ICD2_) && !defined(__MPLAB_ICD3_) && \
   !defined(__MPLAB_PICKIT2__) && !defined(__MPLAB_PICKIT3__) && \
   !defined(__MPLAB_REALICE__) && \
   !defined(__MPLAB_DEBUGGER_REAL_ICE) && \
   !defined(__MPLAB_DEBUGGER_ICD3) && \
   !defined(__MPLAB_DEBUGGER_PK3) && \
   !defined(__MPLAB_DEBUGGER_PICKIT2) && \
   !defined(__MPLAB_DEBUGGER_PIC32MXSK)
    #warning Debug with broken MPLAB simulator
    #define USING_SIMULATOR
#endif

#define FCY 216000000UL    //Oscillator frequency; ricontrollato con baud rate, pare giusto così! (v. anche PLL)
    // ******** per ora 212 overclock confermato 216 21/8/21 v.MUL_54

#define CPU_CLOCK_HZ             (FCY)    // CPU Clock Speed in Hz
#define CPU_CT_HZ            (CPU_CLOCK_HZ/2)    // CPU CoreTimer   in Hz
#define PERIPHERAL_CLOCK_HZ      (FCY/2 /*100000000UL*/)    // Peripheral Bus  in Hz
#define GetSystemClock()         (FCY)    // CPU Clock Speed in Hz
#define GetPeripheralClock()     (PERIPHERAL_CLOCK_HZ)    // Peripheral Bus  in Hz
#define FOSC 8000000UL    //Oscillator frequency

#define US_TO_CT_TICKS  (CPU_CT_HZ/1000000UL)    // uS to CoreTimer Ticks
    
#define VERNUML 2
#define VERNUMH 3

typedef unsigned long UINT32;
typedef unsigned long DWORD;
typedef signed long INT32;
typedef unsigned short int UINT16;
typedef signed short int INT16;
typedef unsigned short int WORD;
//typedef unsigned int size_t;
typedef unsigned char UINT8;
typedef unsigned char BYTE;
typedef signed char INT8;
//typedef enum _BOOL { FALSE = 0, TRUE = 1 } BOOL;    /* Undefined size */

#ifdef SSD1309
#define _TFTWIDTH  		128     //the REAL W resolution of the TFT
#define _TFTHEIGHT 		64      //the REAL H resolution of the TFT
#elif SSD1963
#define _TFTWIDTH  		800
#define _TFTHEIGHT 		480
#else
#define _TFTWIDTH  		160
#define _TFTHEIGHT 		128
#endif

#ifndef SSD1963
typedef signed char GRAPH_COORD_T;
typedef unsigned char UGRAPH_COORD_T;
#else
typedef signed short int GRAPH_COORD_T;
typedef unsigned short int UGRAPH_COORD_T;
#endif

void mySYSTEMConfigPerformance(void);
void myINTEnableSystemMultiVectoredInt(void);

#define ReadCoreTimer()           _CP0_GET_COUNT()           // Read the MIPS Core Timer

#define ClrWdt() { WDTCONbits.WDTCLRKEY=0x5743; }

#define USA_DMA 1
//#define SLIDING_WINDOW 1
// Sampling Control
#define Fosc		FCY
#define Fcy			(Fosc/2)
#define Fs   		3000000UL           // 2560000 questo dà 39 al timer, meno non va :(
// overclock a 216, =>108 multiplo esatto di 2700000
// va a 30000000! 22/8/22 PR1=36, SAMC=3, 8bit, ADCDIV=1
//#define SAMPPRD    (Fcy/Fs)-1
//#define NUMSAMP     560		//http://www.microchip.com/forums/fb.ashx?m=699633
// bandwidth=9KHz => 128 samples per FFT
#define LOG2_BLOCK_LENGTH 	8	// = Number of "Butterfly" Stages in FFT processing 
#define FFT_BLOCK_LENGTH	(1<<LOG2_BLOCK_LENGTH)     // = Number of frequency points in the FFT 
#define SAMPLING_RATE		Fs	// = Rate at which input signal was sampled 
                                // SAMPLING_RATE is used to calculate the frequency
                                // of the largest element in the FFT output vector
//#define INTERESTING_BIN 32      // dove leggiamo output
//#define SUB_BLOCK_LENGTH (FFT_BLOCK_LENGTH/4)
#define LOG2_BLOCK_LENGTH_BF 	4
#define FFT_BLOCK_LENGTH_BF (1<<LOG2_BLOCK_LENGTH_BF)
#define LOG2_BLOCK_LENGTH_BF_2 	6
#define FFT_BLOCK_LENGTH_BF_2 (1<<LOG2_BLOCK_LENGTH_BF_2)

#define SCALING_FACTOR 12
//#define Q1_FACTOR 16 //   // 16.16
#define PWM_MAX 250

typedef q15_t SAMPLE_TYPE;

//#define TEST_SINE_WAVE 1
//#define TEST_SINE_WAVE_MODULATED_AM 1  
//#define TEST_SINE_WAVE_MODULATED_FM 1  


struct __attribute__((__packed__)) SAVED_PARAMETERS {
    uint16_t signature;
    uint8_t MW_LW;      // 1=MW
    uint8_t AM_FM_SSB;  // 0=nulla (tanto per), 1=AM 2=FM 3=SSB 4=LSB 5=USB
    uint8_t waterfall;
    uint8_t spectrum;
    };
extern struct SAVED_PARAMETERS configParms;
void loadSettings(void);
void saveSettings(void);


void ADC_Init(void);
void Timer_Init(void);
void DMA_Init(void);
void PWM_Init(void);
void SPI_Init(void);
void UART_Init(DWORD);
void putsUART1(unsigned int *buffer);

void __attribute__((used)) __delay_ns(unsigned int);
void __attribute__((used)) __delay_us(unsigned int);
void __attribute__((used)) __delay_ms(unsigned int);

//#define round(n) (n+.5)



#ifndef SSD1963

#ifdef PCB2019
#define LED1 LATEbits.LATE2
#define LED2 LATEbits.LATE3
#define LED3 LATEbits.LATE4
#define SW1  PORTDbits.RD2
#define SW2  PORTDbits.RD3
#else
#define LED1 LATBbits.LATB15
#define LED2 LATFbits.LATF4
#define LED3 LATFbits.LATF5
#define SW1  PORTEbits.RE0
#define SW2  PORTEbits.RE1
#endif

#define SPI_232_O		LATGbits.LATG6
#define SPI_232_I		PORTGbits.RG8			//
#define SPI_232_IO		LATGbits.LATG8			//
#define SPI_232_OI		PORTGbits.RG6		// (per I2C clock stretch) 
#define SPI_232_O_TRIS		TRISGbits.TRISG6
#define SPI_232_I_TRIS		TRISGbits.TRISG8
//RG7 può essere SDI2 e RG8 SDO2, per SPI (con RG6=SCK2 fisso))


#define	SPISDITris 0		// niente qua
#define	SPISDOTris TRISGbits.TRISG8				// SDO
#define	SPISCKTris TRISGbits.TRISG6				// SCK
#define	SPICSTris  TRISGbits.TRISG7				// CS
#define	LCDDCTris  TRISEbits.TRISE7				// DC che su questo LCD è "A0" per motivi ignoti
//#define	LCDRSTTris TRISBbits.TRISB7
	
#define	m_SPISCKBit LATGbits.LATG6		// pin 
#define	m_SPISDOBit LATGbits.LATG8		// pin 
#define	m_SPISDIBit 0
#define	m_SPICSBit  LATGbits.LATG7		// pin 
#define	m_LCDDCBit  LATEbits.LATE7 		// pin 
//#define	m_LCDRSTBit LATBbits.LATB7 //FARE
//#define	m_LCDBLBit  LATBbits.LATB12


#else

#define LED1 LATGbits.LATG8
#define LED2 LATGbits.LATG7
//#define LED3 LATFbits.LATF5
#define SW1  PORTEbits.RE5
#define SW2  PORTEbits.RE6
#define SW3  PORTEbits.RE7

#define	m_LCDWRBit  LATFbits.LATF1		// pin 
#define	m_LCDRDBit  LATFbits.LATF0		// pin 
#define	m_LCDRSBit  LATFbits.LATF3 		// pin 
#define	m_LCDCSBit  LATFbits.LATF4 		// pin 

#ifdef SSD1963_SLAVE
#define	SPISDITris TRISDbits.TRISD2				// SDI
//#define	SPISDOTris TRISDbits.TRISD3				// SDO
#define	SPISCKTris TRISDbits.TRISD1				// SCK
#define	SPISSTris  TRISDbits.TRISD4				// SS
#define	SPIRSTris  TRISDbits.TRISD3				// RS
#define	SPI_RS_BIT PORTDbits.RD3
#define	m_SPICSBit  PORTDbits.RD4		// pin 
#else
#define	SPISDITris TRISDbits.TRISD2				// SDO
#define	SPISDOTris TRISDbits.TRISD3				// SDI
#define	SPISCKTris TRISDbits.TRISD1				// SCK
#define	SPICSTris  TRISDbits.TRISD4				// CS
#define	m_SPICSBit  LATDbits.LATD4		// pin 

uint8_t spiXFer(uint8_t);

#endif

#endif

#ifdef	__cplusplus
}
#endif

#endif	/* SDRRADIO32_H */

