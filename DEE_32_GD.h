/************************************************************************
*
*   Emulating Data EEPROM for PIC24 Microcontrollers and
*           dsPIC Digital Signal Controllers
*
*************************************************************************
* FileName:     DEE Emulation 16-bit.h
* Compiler:     MPLAB XC16, v1.30 or higher
* Company:      Microchip Technology, Inc.
*
* Software License Agreement
*
* Copyright © 2016 Microchip Technology Inc. All rights reserved.
*
* Microchip licenses to you the right to use, modify, copy and distribute
* Software only when embedded on a Microchip microcontroller or digital
* signal controller, which is integrated into your product or third party
* product (pursuant to the sublicense terms in the accompanying license
* agreement).
*
* You should refer to the license agreement accompanying this Software for
* additional information regarding your rights and obligations.
*
*
* Author            Date        Comment
*************************************************************************
 * GD/C             2020/7/15   per dsPIC33CH/PIC24
************************************************************************/

#ifndef _DEE_EMULATION_H
#define _DEE_EMULATION_H

// User defined constants
#if defined(__dsPIC33C__) 
#define NUM_DATA_EE_PAGES   1       // 1KWORD
#elif defined(__PIC32MZ__) 
#define NUM_DATA_EE_PAGES   1       // 4KDWORD
#elif defined(__PIC32MM__)
#define NUM_DATA_EE_PAGES   1       // 4KDWORD
#else
#define NUM_DATA_EE_PAGES   2       // 1KWORD
#endif
#define DATA_EE_SIZE        NUMBER_OF_INSTRUCTIONS_IN_PAGE      // /2  //256  // in WORD
#define DATA_EE_TOTAL_SIZE  (NUM_DATA_EE_PAGES * DATA_EE_SIZE)      // ogni istruzione fanno 2 byte usabili

// Modify the following constants based on the specific device being used
#if defined(__dsPIC33E__) || defined(__PIC24E__)
//#define	NUMBER_OF_INSTRUCTIONS_IN_PAGE  512 PORCAMADONNA NON è COSì pic24ep512gp202 (v. DEE del 2014, Larry)); anche CAN GU810
//#define	NUMBER_OF_INSTRUCTIONS_IN_ROW   64
#define	NUMBER_OF_INSTRUCTIONS_IN_PAGE  512 //_FLASH_PAGE         //1024
#define	NUMBER_OF_INSTRUCTIONS_IN_ROW   128         // _FLASH_ROW ??
#elif defined(__dsPIC33C__) 
#define	NUMBER_OF_INSTRUCTIONS_IN_PAGE  _FLASH_PAGE        //1024
//VERIFICARE!!!
#define	NUMBER_OF_INSTRUCTIONS_IN_ROW   128         // _FLASH_ROW ??
#elif defined(__PIC24F__)
#define	NUMBER_OF_INSTRUCTIONS_IN_PAGE  512 //_FLASH_PAGE  //1024
#define	NUMBER_OF_INSTRUCTIONS_IN_ROW   64
// sul PIC24FV32KA302, la riga è di 32 e l'erase può andare da 32 a 96...
#elif defined(__dsPIC33F__) || defined(__PIC24H__)
#define	NUMBER_OF_INSTRUCTIONS_IN_PAGE  512
#define	NUMBER_OF_INSTRUCTIONS_IN_ROW   64
#elif defined(__PIC32MZ__)
#define	NUMBER_OF_INSTRUCTIONS_IN_PAGE  4096 //_FLASH_PAGE  
#define	NUMBER_OF_INSTRUCTIONS_IN_ROW   512
#elif defined(__PIC32MM__)
#define	NUMBER_OF_INSTRUCTIONS_IN_PAGE  4096 //_FLASH_PAGE  
#define	NUMBER_OF_INSTRUCTIONS_IN_ROW   512
#endif

// Uncomment the following line if using a device with Flash ECC feature
#if defined(__dsPIC33C__)           // mah credo... 2020
#define __HAS_ECC	1
#endif

// Uncomment the following line if using Auxiliary Flash for EEPROM Emulation
//#define __AUXFLASH	1

#if defined(__dsPIC33E__) || defined(__PIC24E__) || defined(__dsPIC33C__)

#if defined(__HAS_ECC)
#define ERASE_PAGE          0x4003
#define PROGRAM_ROW         0x4002
#define PROGRAM_WORD        0x4001
#else
#define ERASE_PAGE          0x4003
#define PROGRAM_ROW         0x4002      // sul GP202 non c'è, ma pare che il codice non la usi
#define PROGRAM_WORD        0x4001
#endif

#elif defined(__PIC32MZ__) 


#else

#define ERASE_PAGE          0x4042
#define PROGRAM_ROW         0x4001
#define PROGRAM_WORD        0x4003

#endif

#define ERASE_WRITE_CYCLE_MAX           5
//#define NUMBER_OF_ROWS_IN_PAGE          (_FLASH_PAGE \ _FLASH_ROW)      // non è usata ma cmq non andrebbe "£$%
#define NUMBER_OF_ROWS_IN_PAGE          (NUMBER_OF_INSTRUCTIONS_IN_PAGE \ NUMBER_OF_INSTRUCTIONS_IN_ROW)


extern int  ReadPMHigh(int);
extern int  ReadPMLow(int);
extern void UnlockPM(void);
extern int  WritePMHigh(int, int);
extern int  WritePMLow(int, int);

void            UnlockWrite         (void);
void            ErasePage           (unsigned int addr);
unsigned char   DataEEInit          (void);
unsigned int    DataEERead          (unsigned int addr);
unsigned char   DataEEWrite         (unsigned int data, unsigned int addr);

#endif
