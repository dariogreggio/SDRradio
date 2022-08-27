/************************************************************************
*
*   Emulating Data EEPROM for PIC32 Microcontrollers 
*
* This application note provides a standard interface to an efficient
* Data EEPROM emulation algorithm and uses available program memory. 
* It is designed for Microchip Technology 32-bit PIC devices 
* which currently include PIC32. The project is initially configured to use 
* PIC32MZ2048EFG on the ForgetIvrea Board. 
* User must select number pages of program memory, erase/write limit and 
* emulated DEE size. These are defined in "DEE Emulation 16-bit.h".
* At build-time, the linker reserves pages in the next available 
* locations in program memory. Compiler error occurs if more than 255 
* DEE locations are declared, less than 2 pages of program memory is 
* reserved, greater than 65,535 erase/write cycles specified or if 
* insufficient program memory is available. 
* Call initialization routine and clear status flags before attempting 
* any other DEE operation.
*
*************************************************************************
* FileName:     DEE Emulation 32-bit.c
* Dependencies: Flash Operations.s
*               DEE Emulation 32-bit.h
* Compiler:     MPLAB XC32, v1.30 or higher
* Company:      Dario's Automation, Microchip Technology, Inc.
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
* GD/C              2020/7/15  riscritta come semplice R/W (senza wear leveling) che questa fa cagare :) #gocoronavirusgo

************************************************************************/

#include <xc.h>
#include "Pc_pic_cpu.h"
#include "DEE_32_GD.h"
#include <string.h>
#include <sys/kmem.h>

// User constant validation
#if NUM_DATA_EE_PAGES < 1
    #error Minimum number of program memory pages is 1
#endif

#if ERASE_WRITE_CYCLE_MAX > 65535
    #error Maximum number of erase/write cycles is 65,535
#endif

#define FF_256 \
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" \
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" \
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" \
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" \
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" \
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" \
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" \
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" \
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" \
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" \
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" \
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" \
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" \
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" \
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" \
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" 



//Data EE info stored in PM in following format
//  Status in first two locations of PM page,
//  8-bit DEE Address (odd address, low byte) 16-bit DEE data (even address)
#if defined (__AUXFLASH)

#define DEE_BANK_SIZE (NUMBER_OF_INSTRUCTIONS_IN_PAGE * 2 * NUM_DATA_EE_PAGES)
#define DEE_PAGE_SIZE (NUMBER_OF_INSTRUCTIONS_IN_PAGE * 2)

#define DEE_PAGE_TBL(bank, page) ((0x7FC000 + (DEE_BANK_SIZE * (bank)) + (DEE_PAGE_SIZE * (page))) >> 16)
#define DEE_PAGE_OFFSET(bank, page) ((0x7FC000 + (DEE_BANK_SIZE * (bank)) + (DEE_PAGE_SIZE * (page))) & 0xFFFF)

#else

    const unsigned char emulationPages[NUM_DATA_EE_PAGES][NUMBER_OF_INSTRUCTIONS_IN_PAGE * 4]
        __attribute__((space(prog), aligned(NUMBER_OF_INSTRUCTIONS_IN_PAGE * 4), /*fill(0xff), fillupper(0xff) , noload */ )) = 
    // qua non ci sono fill ecc...
    { { 
    "\x47\x44"
#ifdef USA_EXTENDED_RAM  
    "\x45"    //45
#else
    "\x41"    // numlock-on,ignore errors, usa ext ram, no debug; no verbose; no config; WOL
#endif
    "\x0\x0\x3"   
#if defined(USA_USB_HOST_MSD)
    "\x1\x1"   // C
#else
    "\x1\x0"   // se no se manca IDE si fotte la SD :)
#endif
    "\x1\x1\x1\x1\x1\x1"
#if defined(USA_USB_HOST_UVC)
    "\x1"
#else
    "\x0"
#endif
    "\x1"      // C
    "\x0\x0\x0\x0\x0\x0\x0\x0"      // pasw
    "\x1\x0\x0\x0"
            
    "\xc0\xa8\x01\x51"    // MY_DEFAULT_IP_ADDR_BYTE1 ecc :(
    "\xff\xff\xff\x00"    // MY_DEFAULT_MASK_BYTE1
    "\xc0\xa8\x01\x01"    // MY_DEFAULT_GATE_BYTE1
    "\xc0\xa8\x01\x01"    // MY_DEFAULT_PRIMARY_DNS_BYTE1 
    "\x08\x08\x08\x08"    // MY_DEFAULT_SECONDARY_DNS_BYTE1 
    "\xc0\xa8\x01\x51"    // DefaultIPAddr
    "\x00\xff\xff\xff"    // DefaultMask
    "\x50\x43\x5f\x50\x49\x43\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0"  // NETBios name
    "\x1"   // SNTP
    "\x0\xc"    // network, channel
    "\x00"      // dhcp ecc wifi
    "\x00\x04\xa3\x00\x00\x00"    // SerializedMACAddress ossia MY_DEFAULT_MAC_BYTE1 ecc
    "\x40"      // dhcp ecc Eth
    "\x0"       // video res
            
    "wlan_greggio\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0" 
    "dariog20\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0" 
    "\x0\x0\x0\x0"      // IP idem
    "\x0\x0\x0\x0"      // mask
    "\x0\x0\x0\x0"      // gateway
    "\x0\x0\x0\x0"      // dns
    "\x50\x0\x17\x0"

    "admin\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0" 
    "pasw\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0" 
            
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" 
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" 
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" 
    "\xff\xff\xff\xff\xff\xff\xff\x0" 
//256            
    FF_256
//512            
    FF_256
//768            
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
    FF_256
      }
      
    };
    
   
    #define DEE_BANK_SIZE (sizeof(emulationPages[0])*NUM_DATA_EE_PAGES)
    #define DEE_PAGE_SIZE (sizeof(emulationPages[0]))

    #define DEE_PAGE_TBL(addr) (0x1D000000 | addr)
    #define DEE_PAGE_OFFSET(addr) (0x1D000000 | (unsigned int)&emulationPages | (addr & 0xfffc))

#endif


    
    
void NVMInitiateOperation(void) {
  int int_status; // storage for current Interrupt Enable state
  int dma_susp; // storage for current DMA state
  
  // Disable Interrupts
  asm volatile("di %0" : "=r"(int_status));
  // Disable DMA
#ifndef USING_SIMULATOR
  if(!(dma_susp=DMACONbits.SUSPEND)) {
    DMACONSET=_DMACON_SUSPEND_MASK; // suspend
    while(DMACONbits.DMABUSY); // wait to be actually suspended
    }
#endif
  NVMKEY = 0x0;
  NVMKEY = 0xAA996655;
  NVMKEY = 0x556699AA;
//  NVMCONSET = 1 << 15; // must be an atomic instruction
  NVMCONSET = _NVMCON_WR_MASK;
 // Restore DMA
#ifndef USING_SIMULATOR
  if(!dma_susp) {
    DMACONCLR=_DMACON_SUSPEND_MASK; // resume DMA activity
    }
#endif
  // Restore Interrupts
  if(int_status & 0x00000001) {
    asm volatile("ei");
    }
  }

/************************************************************************
ErasePage

This routine erases the selected page.

Parameters:		Page number
Return:			None
Side Effects:	Loads NVCOM with erase opcode
************************************************************************/
void ErasePage(unsigned int addr) {

  NVMADDR = KVA_TO_PA(DEE_PAGE_OFFSET(addr));
  //1d012000

  NVMCONbits.NVMOP = 0x4;

// Enable Flash Write
//  NVMCONbits.WREN = 1;
  NVMCONSET = _NVMCON_WREN_MASK;    // 

  NVMInitiateOperation();
//  NVMKEY= 0;
//  NVMKEY = FLASH_PROGRAM_UNLOCK_KEY1;
//  NVMKEY = FLASH_PROGRAM_UNLOCK_KEY2;

//  NVMCONSET = _NVMCON_WREN_MASK;
  while(NVMCON & _NVMCON_WR_MASK);
  // Wait for WR bit to clear
//      while(NVMCONbits.WR);

  NVMCONCLR = _NVMCON_WREN_MASK;

  if(NVMCON & 0x3000) {
    // fare Page Erase Retry, v. datasheet/doc
//??    NVMADDR = addr;
//    NVMCONbits.NVMOP = 0x1;
//    NVMCONSET = _NVMCON_WREN_MASK;
    }

  }


/************************************************************************
DataEEInit

This function must be called prior to any other operation.

Parameters:		None
Return:			Status value (0 for pass)
Side Effects:	
************************************************************************/
unsigned char DataEEInit(void) {
    
  return 0;
	}

/************************************************************************
DataEERead

This routine verifies the address is valid. If not, the Illegal Address
flag is set and 0xFFFF is returned. It then finds the active page. If an
active page can not be found, the Page Corrupt status bit is set and
0xFFFF is returned. A reverse search of the active page attempts to find
the matching address in the program memory MSB (odd address). If a match
is found, the corresponding data EEPROM data (even address) is returned,
otherwise 0xFFFF is returned. This function can be called by the user.

Parameters:		Data EE address
Return:			Data EE data or 0xFFFF if address not found
Side Effects:	Data EE flags may be updated.
************************************************************************/
unsigned int DataEERead(unsigned int addr) {
  unsigned int i;
  uint32_t *p;

//i=emulationPages[0][(addr*2+1) % (NUMBER_OF_INSTRUCTIONS_IN_PAGE*2)];
//i <<= 8;
//i|=emulationPages[0][(addr*2) % (NUMBER_OF_INSTRUCTIONS_IN_PAGE*2)];
//return i;

  addr *=4;   // un'istruzione = 4 byte usabili!
  addr %= DATA_EE_TOTAL_SIZE*4;   // wrap around, mi piace (ergo x4 qua)

//  p=0x1D000000 | addr;
  p=(uint32_t *)DEE_PAGE_OFFSET(addr);

  i=*p;
  return i;  
	}

/************************************************************************
DataEEWrite

This routine verifies the address is valid. If not, the Illegal Address
flag is set and an error code is returned. It then finds the active page.
If an active page can not be found, the Page Corrupt status bit is set
and an error code is returned. A read is performed, if the data was not
changed, the function exits. If the last location is programmed, the Pack
Skipped error flag is set (one location should always be available). The
data EE information (MSB = address, LSW = data) is programmed and
verified. If the verify fails, the Write Error flag is set. If the write
went into the last location of the page, pack is called. This function
can be called by the user.

Parameters:		Data EE address and data
Return:			Pass or fail status (0 = Pass)
Side Effects:	Data EE flags may be updated. CPU stall occurs for flash
                programming. Pack may be generated.
************************************************************************/
unsigned char DataEEWrite(unsigned int data, unsigned int addr) {
  unsigned int i,j;
  uint32_t *p;
  unsigned int emulationPagesShadow[NUMBER_OF_INSTRUCTIONS_IN_PAGE]
    __attribute__((aligned(NUMBER_OF_INSTRUCTIONS_IN_PAGE)));   // 
  

  //Do not write data if it did not change
  i=DataEERead(addr);
  if(i == data) {
//    TBLPAG = savedTBLPAG;
    return 0;
    }

  addr *=4;   // un'istruzione = 4 byte usabili!
  addr %= DATA_EE_TOTAL_SIZE*4;   // wrap around, mi piace (ergo x4 qua)

  if((i & data) != data) {    // se devo cancellare...

  //  j=-NUMBER_OF_INSTRUCTIONS_IN_PAGE*2;
    j=addr & /*0xfc00*/ -(NUMBER_OF_INSTRUCTIONS_IN_PAGE*4);
    for(i=0; i<NUMBER_OF_INSTRUCTIONS_IN_PAGE; i++) {
//      p=0x1D008000 | i;
      p=(uint32_t *)DEE_PAGE_OFFSET(j+i*4);
      emulationPagesShadow[i]=   i /* *p*/;
      }

    emulationPagesShadow[(addr/4) % (NUMBER_OF_INSTRUCTIONS_IN_PAGE)]=data;

  //  WritePMLow(data, pmOffset);
  //  WritePMHigh((addr % DATA_EE_SIZE), pmOffset);

    ErasePage(addr);
    
/*  rowbuff[0]=0x1111;
  rowbuff[1]=0x2222;
  rowbuff[2]=0x3333;
  rowbuff[3]=0x4444;
  NVMADDR = 0x1D008000;
  NVMSRCADDR = (unsigned int)((int)rowbuff & 0x1FFFFFFF);
  NVMCONbits.NVMOP = 0x3; // NVMOP for Row programming
  NVMCONbits.WREN = 1;
  NVMInitiateOperation(); // see Example 52-1
  while(NVMCONbits.WR);
  NVMCONbits.WREN = 0;
 * */

    for(i=0; i<NUMBER_OF_INSTRUCTIONS_IN_PAGE; i+=NUMBER_OF_INSTRUCTIONS_IN_ROW) {
      j=addr & /*0xfc00*/ -(NUMBER_OF_INSTRUCTIONS_IN_PAGE*4);
      NVMADDR  = KVA_TO_PA(DEE_PAGE_OFFSET(j+i*4));     // physical address 
      NVMSRCADDR = KVA_TO_PA((unsigned int)&emulationPagesShadow[i]);     // physical address 

      NVMCONbits.NVMOP = 0x3; // NVMOP for Row programming
      // Enable Flash for write operation and set the NVMOP 
//      NVMCONbits.WREN = 1;  // Start programming
      NVMCONSET = _NVMCON_WREN_MASK;    // 
      NVMInitiateOperation();   // see Example 52-1
      // Wait for WR bit to clear
      while(NVMCONbits.WR);
      // Disable future Flash Write/Erase operations
      NVMCONbits.WREN = 0;
      // Check Error Status
      if(NVMCON & 0x3000)       // mask for WRERR and LVDERR
        {
        // process errors
        }
      }
    
#if 0 // meglio usare row page... anche per lentezza    
    for(i=0; i<NUMBER_OF_INSTRUCTIONS_IN_PAGE; i++) {
      NVMADDR  = KVA_TO_PA(DEE_PAGE_OFFSET(i));     // physical address 
      NVMDATA0 = emulationPagesShadow[i];     // value

      NVMCONbits.NVMOP = 0x1;   // NVMOP for Word programming
      // Enable Flash for write operation and set the NVMOP 
//      NVMCONbits.WREN = 1;  // Start programming
      NVMCONSET = _NVMCON_WREN_MASK;    // dice TASSATIVO usare atomico!
      NVMInitiateOperation();   // see Example 52-1
      // Wait for WR bit to clear
      while(NVMCONbits.WR);
      // Disable future Flash Write/Erase operations
      NVMCONbits.WREN = 0;
      // Check Error Status
      if(NVMCON & 0x3000)       // mask for WRERR and LVDERR
        {
        // process errors
        }
      }
#endif
    }
  else {    // scrivo soltanto!
    NVMADDR  = KVA_TO_PA(DEE_PAGE_OFFSET(addr));     // physical address 
    NVMDATA0 = data;     // value

  // set the operation, assumes WREN = 0
    NVMCONbits.NVMOP = 0x1;   // NVMOP for Word programming
    // Enable Flash for write operation and set the NVMOP 
//    NVMCONbits.WREN = 1;  // Start programming
    NVMCONSET = _NVMCON_WREN_MASK;    // 
    NVMInitiateOperation();   // see Example 52-1
    // Wait for WR bit to clear
    while(NVMCONbits.WR);
    // Disable future Flash Write/Erase operations
    NVMCONbits.WREN = 0;
    // Check Error Status
    if(NVMCON & 0x3000)       // mask for WRERR and LVDERR
      {
      // process errors
      }
    }


  return 0;
	}


unsigned char AreaEEWrite(void *data, unsigned int len) {
	unsigned int addr,p;
  unsigned char EEbuffer[NUMBER_OF_INSTRUCTIONS_IN_ROW*4] 
    __attribute__((aligned(NUMBER_OF_INSTRUCTIONS_IN_PAGE * 4))); // non so se PAGE o ROW... così va
  // (coherent),


  addr =0;   // un'istruzione = 4 byte usabili!

  ErasePage(addr);

  p=DEE_PAGE_OFFSET(addr);
  
	len /= NUMBER_OF_INSTRUCTIONS_IN_ROW*4;

  NVMADDR  = KVA_TO_PA(p);     // physical address 
  
	
//	while(len--) {    // se più di una... 2KB
  memcpy(EEbuffer,data,/*len +*/ (NUMBER_OF_INSTRUCTIONS_IN_ROW*4));  // serve allineato...
  
#if 0
	unsigned int cp0;
  cp0 = _mfc0(16, 0);
  cp0 &= ~0x07;		// 03 negli esempi...
  cp0 |= /*UNCACHED=*/0x02; // 
  _mtc0(16, 0, cp0);  
#endif
  
/* Set __PIC32_CACHE_MODE to the desired coherency attribute */
//#define __PIC32_CACHE_MODE      _CACHE_WRITEBACK_WRITEALLOCATE    //default runtime setup policy
//#define __PIC32_CACHE_MODE      _CACHE_DISABLE

#ifndef USING_SIMULATOR
    SYS_DEVCON_DataCacheFlush();
    SYS_DEVCON_InstructionCacheFlush();
#endif
    //SYS_DEVCON_CacheClean();
//  __delay_ms(100);     // (pare serva un ritardo ) perché c'è la cache WB (se c'è) opp DISATTIVARLA??
  
		NVMSRCADDR=KVA_TO_PA(EEbuffer);     // value

  	// set the operation, assumes WREN = 0
		NVMCONbits.NVMOP = 0b0011;   // NVMOP for Row programming
		// Enable Flash for write operation and set the NVMOP 
		NVMCONSET = _NVMCON_WREN_MASK;    // 
		NVMInitiateOperation();   // see Example 52-1
		// Wait for WR bit to clear
		while(NVMCONbits.WR);
		// Disable future Flash Write/Erase operations
		NVMCONCLR = _NVMCON_WREN_MASK;    // 
		// Check Error Status
		if(NVMCON & 0x3000)       // mask for WRERR and LVDERR
			{
			return -1;
			}
//		}
  
    /*
    printf("NVMADDR %X\r\n",NVMADDR);
    printf("NVMSRCADDR %X %X\r\n",NVMSRCADDR,data);
__delay_ms(2000);
*/

	return 0;
	}


/*
void MPU_Cache_Flush(void) {
   int index;

   // WriteBack and Invalidate data cache
   for(index=0; index<4096; index+=16) 
     _cache(Index_WriteBack_Invalidate_D,(char*)Index);

   // Invalidate instruction cache
   for(index=0; index<16384; index+=16) 
     _cache(Index_Invalidate_I,(char*)Index);
  }
*/
          
          