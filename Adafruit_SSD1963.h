#ifndef _SSD1963_H
#define _SSD1963_H

#include <xc.h>
#include <stdint.h>
#include "generictypedefs.h"

// the SSD1963 controller can be connected to the TFT screen with 18 or 24 data lines. So we need to tell the constructor which one.
#define TFTBUS18 18
#define TFTBUS24 24
#define TFT_SSD1963 TFTBUS24

//#define DATA_BUS_24 1
#define DATA_BUS_16 1           //16bit 565
//#define DATA_BUS_SPI 1        // 

//typedef uint32_t COLORREF;

enum {
        SSD1963_NOP                   = 0x00,
        SSD1963_SoftReset             = 0x01,
        SSD1963_GetPowerMode          = 0x0A,
        SSD1963_GetAddressMode        = 0x0B,
        SSD1963_GetDisplayMode        = 0x0D,
        SSD1963_GetTearEffectsStatus  = 0x0E,
        SSD1963_EnterSleepMode        = 0x10,
        SSD1963_ExitSleepMode         = 0x11,
        SSD1963_EnterPartialMode      = 0x12,
        SSD1963_EnterNormalMode       = 0x13,
        SSD1963_ExitInvertMode        = 0x20,
        SSD1963_EnterInvertMode       = 0x21,
        SSD1963_SetGammaCurve         = 0x26,
        SSD1963_SetDisplayOff         = 0x28,
        SSD1963_SetDisplayOn          = 0x29,
        SSD1963_SetColumnAddress      = 0x2A,
        SSD1963_SetPageAddress        = 0x2B,
        SSD1963_WriteMemoryStart      = 0x2C,
        SSD1963_ReadMemoryStart       = 0x2E,
        SSD1963_SetPartialArea        = 0x30,
        SSD1963_SetScrollArea         = 0x33,
        SSD1963_SetTearOff            = 0x34,
        SSD1963_SetTearOn             = 0x35,
        SSD1963_SetAddressMode        = 0x36,
        SSD1963_SetScrollStart        = 0x37,
        SSD1963_ExitIdleMode          = 0x38,
        SSD1963_EnterIdleMode         = 0x39,
        SSD1963_WriteMemoryContinue   = 0x3C,
        SSD1963_ReadMemoryContinue    = 0x3E,
        SSD1963_SetTearScanline       = 0x44,
        SSD1963_GetScanline           = 0x45,
        SSD1963_ReadDDB               = 0xA1,
        SSD1963_SetLCDMode            = 0xB0,
        SSD1963_GetLCDMode            = 0xB1,
        SSD1963_SetHoriPeriod         = 0xB4,
        SSD1963_GetHoriPeriod         = 0xB5,
        SSD1963_SetVertPeriod         = 0xB6,
        SSD1963_GetVertPeriod         = 0xB7,
        SSD1963_SetGPIOConf           = 0xB8,
        SSD1963_GetGPIOConf           = 0xB9,
        SSD1963_SetGPIOValue          = 0xBA,
        SSD1963_GetGPIOValue          = 0xBB,
        SSD1963_SetPostProc           = 0xBC,
        SSD1963_GetPostProc           = 0xBD,
        SSD1963_SetPWMConf            = 0xBE,
        SSD1963_GetPWMConf            = 0xBF,
        SSD1963_SetLCDGen0            = 0xC0,
        SSD1963_GetLCDGen0            = 0xC1,
        SSD1963_SetLCDGen1            = 0xC2,
        SSD1963_GetLCDGen1            = 0xC3,
        SSD1963_SetLCDGen2            = 0xC4,
        SSD1963_GetLCDGen2            = 0xC5,
        SSD1963_SetLCDGen3            = 0xC6,
        SSD1963_GetLCDGen3            = 0xC7,
        SSD1963_SetGPIO0Rop           = 0xC8,
        SSD1963_GetGPIO0Rop           = 0xC9,
        SSD1963_SetGPIO1Rop           = 0xCA,
        SSD1963_GetGPIO1Rop           = 0xCB,
        SSD1963_SetGPIO2Rop           = 0xCC,
        SSD1963_GetGPIO2Rop           = 0xCD,
        SSD1963_SetGPIO3Rop           = 0xCE,
        SSD1963_GetGPIO3Rop           = 0xCF,
        SSD1963_SetDBCConf            = 0xD0,
        SSD1963_GetDBCConf            = 0xD1,
        SSD1963_SetDBCTh              = 0xD4,
        SSD1963_GetDBCTh              = 0xD5,
        SSD1963_SetPLL                = 0xE0,
        SSD1963_SetPLLMN              = 0xE2,
        SSD1963_GetPLLMN              = 0xE3,
        SSD1963_GetPLLStatus          = 0xE4,
        SSD1963_SetDeepSleep          = 0xE5,
        SSD1963_SetLShiftFreq         = 0xE6,
        SSD1963_GetLShiftFreq         = 0xE7,
        SSD1963_SetPixelDataInterface = 0xF0,
        SSD1963_GetPixelDataInterface = 0xF1
	};

#define SSD1963_EMODE 0x0003
#define SSD1963_TRI   0x8000
#define SSD1963_DFM   0x4000
#define SSD1963_BGR   0x1000
#define SSD1963_HWM   0x0200
#define SSD1963_ORG   0x0080
#define SSD1963_ID0   0x0000
#define SSD1963_ID1   0x0010
#define SSD1963_ID2   0x0020
#define SSD1963_ID3   0x0030
#define SSD1963_AM    0x0008

#define   HDP 799
#define   HT 928
#define   HPS 46
#define   LPS 15
#define   HPW 48

#define   VDP 479
#define   VT 525
#define   VPS 16
#define   FPS 8
#define   VPW 16


        /*static*/ void SSD1963_command(uint16_t cmd);
        /*static*/ void SSD1963_data8(uint8_t cmd);
        /*static*/ void SSD1963_data(uint16_t cmd);
        uint16_t SSD1963_read(void);
        void SSD1963_initInterface(void);


        enum {
            TFT7         = 0x01, // 7" TFT Screen
            VGA640       = 0x02, // 640x480 VGA
            VGA720       = 0x03, // 720x400 9x16 text mode
            MIKROMEDIA   = 0x04  // MikroElektronika MikroMedia MX7
            };

        void SSD1963_1(
            uint8_t tft_bus_width /*= TFTBUS18*/
            );

        void SSD1963_2(
            uint8_t tft_bus_width /*= TFTBUS18*/
            );


        void SSD1963_fillScreen(UINT16 color);
        void SSD1963_setPixel(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UINT16 color);
        void write24BitColor(UINT32 color);
        BOOL boundaryCheck(UGRAPH_COORD_T ,UGRAPH_COORD_T );
        void __attribute__((always_inline)) HLine(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UINT16 color);
        void __attribute__((always_inline)) VLine(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T h, UINT16 color);

        void SSD1963_drawVerticalLine(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T h, UINT16 color);
        void SSD1963_drawHorizontalLine(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UINT16 color);
        void fillRectangle(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UGRAPH_COORD_T h, UINT16 color);
        void SSD1963_invertDisplay(unsigned char i);
        void displayOn(void);
        void displayOff(void);
        
void _swap(UGRAPH_COORD_T *, UGRAPH_COORD_T *);

        UINT16 colorAt(UGRAPH_COORD_T x, UGRAPH_COORD_T y);

        void SSD1963_initializeDevice(void);

        void SSD1963_initializeDevice2(uint8_t p);

        void SSD1963_windowData(UINT16 d);
        void SSD1963_windowData2(const UINT16 *d, int num);
        void SSD1963_openWindow(UGRAPH_COORD_T, UGRAPH_COORD_T, UGRAPH_COORD_T, UGRAPH_COORD_T);
        void SSD1963_closeWindow(void);

        void SSD1963_enableBacklight(void);
        void SSD1963_disableBacklight(void);
        void SSD1963_setBacklight(uint8_t b);


      void SSD1963_PORTB(
         uint8_t tft_bus_width /*= TFTBUS18*/
         );

    uint8_t drawJPEG(UGRAPH_COORD_T x,UGRAPH_COORD_T y,UGRAPH_COORD_T xs,UGRAPH_COORD_T ys,
        const uint8_t *image,uint32_t imagesize);
    uint8_t readArea(UGRAPH_COORD_T left,UGRAPH_COORD_T top,
        UGRAPH_COORD_T right,UGRAPH_COORD_T bottom,uint16_t *area);
    uint8_t putArea(UGRAPH_COORD_T left,UGRAPH_COORD_T top,
        UGRAPH_COORD_T right,UGRAPH_COORD_T bottom,const uint16_t *area);
    void scrollArea(UGRAPH_COORD_T x1, UGRAPH_COORD_T y1, UGRAPH_COORD_T x2, UGRAPH_COORD_T y2, 
        int16_t dx, int16_t dy);

#endif


