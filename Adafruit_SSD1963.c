/*

	G.Dar 2022, based upon some "cariad" :)
    ( https://github.com/CariadDisplayLibrary/SSD1963/tree/master/src )
		(with a look at old torchio/agie)

	Draghi/17 is over!

*/


#include "sdrradio32.h"
#include "Adafruit_SSD1963.h"
#include "Adafruit_GFX.h"

#define DELAY_LCD() { Nop(); /*Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();*/ }

static uint8_t _profile;
#define SSD1963_TFTWIDTH 800 // [Don't use this any more!]
#define SSD1963_TFTHEIGHT 480 // [Don't use this any more!]
static uint8_t bus_width;
static uint8_t _brightness;
uint8_t rotation;
WORD _xstart,_ystart;     // v. altri ST77xx .. usato per shiftare finestra..


void SSD1963_SSD1963(uint8_t tft_bus_width) {

  bus_width = tft_bus_width;
  _profile=TFT7;
  
  Adafruit_GFX(SSD1963_TFTWIDTH, SSD1963_TFTHEIGHT);
	}

void SSD1963_command(uint16_t cmd) {

#ifdef NO_LCD_DEBUG
  return;
#endif
  
#if TFT_SSD1963 == TFTSPI
  m_LCDRSBit = 0;
  m_LCDCSBit = 0;
  spiWrite(cmd);
  m_LCDCSBit = 1;
#else
  m_LCDRSBit = 0;
  LATB=cmd;
  m_LCDCSBit = 0;
#ifdef m_LCDWRBit
  m_LCDWRBit=0;
  DELAY_LCD();
  m_LCDWRBit=1;
#endif
  m_LCDCSBit = 1;
#endif
	}

void SSD1963_data8(uint8_t d) {

#ifdef NO_LCD_DEBUG
  return;
#endif
  
#if TFT_SSD1963 == TFTSPI
  m_LCDRSBit = 1;
  m_LCDCSBit = 0;
  spiWrite(cmd);
  m_LCDCSBit = 1;
#else
  m_LCDRSBit = 1;
  LATB= (LATB & 0xffffffff00000000) | d;
  m_LCDCSBit = 0;
#ifdef m_LCDWRBit
  m_LCDWRBit=0;
  DELAY_LCD();
  m_LCDWRBit=1;
#endif
  m_LCDCSBit = 1;
#endif
	}

void SSD1963_data(uint16_t d) {

  #ifdef NO_LCD_DEBUG
  return;
#endif

#if TFT_SSD1963 == TFTSPI
  m_LCDRSBit = 1;
  m_LCDCSBit = 0;
  spiWrite(cmd);
  m_LCDCSBit = 1;
#else
  m_LCDRSBit = 1;
  LATB=d;
  m_LCDCSBit = 0;
#ifdef m_LCDWRBit
    m_LCDWRBit=0;
  DELAY_LCD();
    m_LCDWRBit=1;
#endif
  m_LCDCSBit = 1;
#endif
	}

void SSD1963_data_raw(uint16_t d) {

#ifdef NO_LCD_DEBUG
  return;
#endif
  
  LATB=d;
  m_LCDWRBit=0;
  DELAY_LCD();
  m_LCDWRBit=1;
	}

uint16_t SSD1963_read(void) {

#if TFT_SSD1963 == TFTSPI
  m_LCDRSBit = 1;
  m_LCDCSBit = 0;
// qua???  spiWrite(cmd);
  uint16_t out = 0xffff;
  m_LCDCSBit = 1;
#else
  TRISB=0xffff;

  // QUA SERVONO dei ritardi (v. anche forum...)
  m_LCDRSBit = 1;
//  __delay_us(1);
  m_LCDCSBit = 0;
  DELAY_LCD();
//  __delay_ns(100);
#ifdef m_LCDRDBit
  m_LCDRDBit=0;
#endif
  __delay_ns(30);     // meno di 30 legge male...
  uint16_t out = PORTB;
  DELAY_LCD();
//  __delay_us(1);

#ifdef m_LCDRDBit
  m_LCDRDBit=1;
#endif
  TRISB=0x0000;
  m_LCDCSBit = 1;
#endif

  return out;
  }

void SSD1963_initInterface(void) {

  m_LCDRSBit=0;
  
#if TFT_SSD1963 == TFTSPI
  
  initSPI(SPI_DEFAULT_FREQ, spiMode);
  
#else
  
  ANSELB = 0x0000;
//  ANSELF = 0x0000;
  
  TRISFbits.TRISF3=0;
  TRISFbits.TRISF4=0;
#ifdef m_LCDRDBit
  TRISFbits.TRISF0=0;
  TRISFbits.TRISF1=0;
#endif
#ifdef m_LCDResBit
  pinMode(pin_reset, OUTPUT);
#endif
    

  m_LCDCSBit=1;
  m_LCDRSBit=1;
#ifdef m_LCDRDBit
  m_LCDRDBit=1;
  m_LCDWRBit=1;
#endif
  
  TRISB=0x0000;


#ifdef m_LCDResBit
  pinMode(pin_reset, OUTPUT);
  m_LCDResBit=1;
  __delay_ms(100);
  m_LCDResBit=0;
  __delay_ms(100);
  m_LCDResBit=1;

#else
    
    
#endif
#endif
    
  __delay_ms(100);

	}

void SSD1963_initializeDevice2(uint8_t p) {
  
  _profile = p;
  SSD1963_initializeDevice();
  }

void SSD1963_initializeDevice(void) {

  SSD1963_initInterface();

  SSD1963_command(SSD1963_SetPLLMN);
  SSD1963_data8(0x23);        //N=0x36 for 6.5M, 0x23 for 10M crystal
  SSD1963_data8(0x02);
  SSD1963_data8(0x04);
  SSD1963_command(SSD1963_SetPLL);  // PLL enable
  SSD1963_data8(0x01);
  __delay_ms(1);		// ms o us? bah
  SSD1963_command(SSD1963_SetPLL);
  SSD1963_data8(0x03);
  __delay_ms(5);
  SSD1963_command(SSD1963_SoftReset);  // software reset
  __delay_ms(5);

  SSD1963_command(SSD1963_SetLCDMode);
  // The SSD1963 controller can be connected to the TFT screen with 18 or 24 data lines.
#if TFT_SSD1963 == TFTBUS24
    SSD1963_data8(0x20);
#elif TFT_SSD1963 == TFTBUS18
    SSD1963_data8(0x00);
#else
  SSD1963_data8(0x20);   // è il bus verso il vetro, non quello I/F 
#endif
  SSD1963_data8(0x00);


    // Set the timing for the different profiles.

    // 7 inch TFT profile

    switch(_profile) {
      case TFT7:
        _width  = SSD1963_TFTWIDTH /*800*/;
        _height = SSD1963_TFTHEIGHT /*480*/;

        // Continues on from previous SetLCDMode command
        SSD1963_data8(HIBYTE(HDP));  //Set HDP
        SSD1963_data8(LOBYTE(HDP));
        SSD1963_data8(HIBYTE(VDP));  //Set VDP
        SSD1963_data8(LOBYTE(VDP));
        SSD1963_data8(0x00);

        SSD1963_command(SSD1963_SetHoriPeriod);
        SSD1963_data8(HIBYTE(HT));  //Set HT
        SSD1963_data8(LOBYTE(HT));
        SSD1963_data8(HIBYTE(HPS));  //Set HPS
        SSD1963_data8(LOBYTE(HPS));
        SSD1963_data8(HPW);              //Set HPW
        SSD1963_data8(HIBYTE(LPS));  //Set HPS
        SSD1963_data8(LOBYTE(LPS));
        SSD1963_data8(0x00);

        SSD1963_command(SSD1963_SetVertPeriod);
        SSD1963_data8(HIBYTE(VT));   //Set VT
        SSD1963_data8(LOBYTE(VT));
        SSD1963_data8(HIBYTE(VPS));  //Set VPS
        SSD1963_data8(LOBYTE(VPS));
        SSD1963_data8(VPW);              //Set VPW
        SSD1963_data8(HIBYTE(FPS));  //Set FPS
        SSD1963_data8(LOBYTE(FPS));

        SSD1963_command(SSD1963_SetLShiftFreq); //PLL setting for PCLK, depends on resolution
        SSD1963_data8(0x03);
        SSD1963_data8(0xff);
        SSD1963_data8(0xff);

        SSD1963_command(SSD1963_SetAddressMode); //rotation
        SSD1963_data8(0x00); // RGB

        break;
      case MIKROMEDIA:
        _width  = 480;
        _height = 272;

        // Continues on from previous SetLCDMode command
        SSD1963_data8(HIBYTE(479));  //Set HDP
        SSD1963_data8(LOBYTE(479));
        SSD1963_data8(HIBYTE(271));  //Set VDP
        SSD1963_data8(LOBYTE(271));
        SSD1963_data8(0x00);

        SSD1963_command(SSD1963_SetHoriPeriod);
        SSD1963_data8(HIBYTE(531));  //Set HT
        SSD1963_data8(LOBYTE(531));
        SSD1963_data8(HIBYTE(43));  //Set HPS
        SSD1963_data8(LOBYTE(43));
        SSD1963_data8(10);              //Set HPW
        SSD1963_data8(HIBYTE(8));  //Set HPS
        SSD1963_data8(LOBYTE(8));
        SSD1963_data8(0x00);

        SSD1963_command(SSD1963_SetVertPeriod);
        SSD1963_data8(HIBYTE(288));   //Set VT
        SSD1963_data8(LOBYTE(288));
        SSD1963_data8(HIBYTE(12));  //Set VPS
        SSD1963_data8(LOBYTE(12));
        SSD1963_data8(10);              //Set VPW
        SSD1963_data8(HIBYTE(4));  //Set FPS
        SSD1963_data8(LOBYTE(4));

        SSD1963_command(SSD1963_SetLShiftFreq); //PLL setting for PCLK, depends on resolution
        SSD1963_data8(0x03);
        SSD1963_data8(0xff);
        SSD1963_data8(0xff);

        SSD1963_command(SSD1963_SetAddressMode); //rotation
        SSD1963_data8(0x00); // RGB

        break;
      case VGA640:
      {
        _width  = 640;
        _height = 480;
        uint16_t hdp = 640 - 1; // panel width
        uint16_t vdp = 480 - 1; // panel height

        uint16_t ht = 800 - 1; // total pixel clocks per line
        uint16_t hps = 96+48 - 1 + 16; // Blank period
        uint16_t hpw = 96 - 1;  // Sync width
        uint16_t lps = 16 - 1;  // Front porch

        uint16_t vt = 525 - 1;
        uint16_t vps = 2+33 - 1 + 10; // Blank period
        uint16_t vpw = 2 - 1;
        uint16_t fps = 10 - 1;  // Front porch

        // Continues on from previous SetLCDMode command
        SSD1963_data8(HIBYTE(hdp));  //Set HDP
        SSD1963_data8(LOBYTE(hdp));
        SSD1963_data8(HIBYTE(vdp));  //Set VDP
        SSD1963_data8(LOBYTE(vdp));
        SSD1963_data8(0x00);

        SSD1963_command(SSD1963_SetHoriPeriod);
        SSD1963_data8(HIBYTE(ht));  //Set HT
        SSD1963_data8(LOBYTE(ht));
        SSD1963_data8(HIBYTE(hps));  //Set HPS
        SSD1963_data8(LOBYTE(hps));
        SSD1963_data8(hpw);              //Set HPW
        SSD1963_data8(HIBYTE(lps));  //Set HPS
        SSD1963_data8(LOBYTE(lps));
        SSD1963_data8(0x00);

        SSD1963_command(SSD1963_SetVertPeriod);
        SSD1963_data8(HIBYTE(vt));   //Set VT
        SSD1963_data8(LOBYTE(vt));
        SSD1963_data8(HIBYTE(vps));  //Set VPS
        SSD1963_data8(LOBYTE(vps));
        SSD1963_data8(vpw);              //Set VPW
        SSD1963_data8(HIBYTE(fps));  //Set FPS
        SSD1963_data8(LOBYTE(fps));

        SSD1963_command(SSD1963_SetLShiftFreq); //PLL setting for PCLK, depends on resolution
        // 73Hz (steady as a rock)
        SSD1963_data8(0x03);
        SSD1963_data8(0x68);
        SSD1963_data8(0x1a);
/*
        // 60Hz (has a shimmer on my monitor)
        SSD1963_data8(0x02);
        SSD1963_data8(0xCC);
        SSD1963_data8(0x16);
*/

        SSD1963_command(SSD1963_SetAddressMode); //rotation
        SSD1963_data8(0x0008); // BGR
      }
        break;
      case VGA720:
      {
        _width  = 720;
        _height = 400;
        const uint16_t hdp = 720 - 1; // panel width
        const uint16_t vdp = 400 - 1; // panel height

        const uint16_t h_fp = 36;
        const uint16_t h_pw = 72;
        const uint16_t h_bp = 108;

        const uint16_t ht = 936 - 1; // total pixel clocks per line
        const uint16_t hps = h_fp + h_pw + h_bp - 1; // Blank period
        const uint16_t hpw = h_pw - 1;  // Sync width
        const uint16_t lps = h_fp - 1;  // Front porch

        const uint16_t v_fp = 1;
        const uint16_t v_pw = 3;
        const uint16_t v_bp = 42;

        uint16_t vt = 446 - 1;
        uint16_t vps = v_fp + v_pw + v_bp - 1; // Blank period
        uint16_t vpw = v_pw - 1;
        uint16_t fps = v_fp - 1;  // Front porch

        // Continues on from previous SetLCDMode command
        SSD1963_data8(HIBYTE(hdp));  //Set HDP
        SSD1963_data8(LOBYTE(hdp));
        SSD1963_data8(HIBYTE(vdp));  //Set VDP
        SSD1963_data8(LOBYTE(vdp));
        SSD1963_data8(0x00);

        SSD1963_command(SSD1963_SetHoriPeriod);
        SSD1963_data8(HIBYTE(ht));  //Set HT
        SSD1963_data8(LOBYTE(ht));
        SSD1963_data8(HIBYTE(hps));  //Set HPS
        SSD1963_data8(LOBYTE(hps));
        SSD1963_data8(hpw);              //Set HPW
        SSD1963_data8(HIBYTE(lps));  //Set HPS
        SSD1963_data8(LOBYTE(lps));
        SSD1963_data8(0x00);

        SSD1963_command(SSD1963_SetVertPeriod);
        SSD1963_data8(HIBYTE(vt));   //Set VT
        SSD1963_data8(LOBYTE(vt));
        SSD1963_data8(HIBYTE(vps));  //Set VPS
        SSD1963_data8(LOBYTE(vps));
        SSD1963_data8(vpw);              //Set VPW
        SSD1963_data8(HIBYTE(fps));  //Set FPS
        SSD1963_data8(LOBYTE(fps));

        SSD1963_command(SSD1963_SetLShiftFreq); //PLL setting for PCLK, depends on resolution
//        // 85Hz
//        SSD1963_data8(0x03);
//        SSD1963_data8(0xf0);
//        SSD1963_data8(0x00);
        // 70Hz
        SSD1963_data8(0x03);
        SSD1963_data8(0x30);
        SSD1963_data8(0x00);

        // 73Hz (steady as a rock)
//        SSD1963_data8(0x0003);
//        SSD1963_data8(0x0068);
//        SSD1963_data8(0x001a);

        // 60Hz (has a shimmer on my monitor)
//        SSD1963_data8(0x0002);
//        SSD1963_data8(0x00CC);
//        SSD1963_data8(0x0016);


        SSD1963_command(SSD1963_SetAddressMode); //rotation
        SSD1963_data8(0x08); // BGR
      }
      break;
    }

    SSD1963_command(SSD1963_SetGPIOValue);
    SSD1963_data8(0x05);    //GPIO[3:0] out 1

    SSD1963_command(SSD1963_SetGPIOConf);
    SSD1963_data8(0x07);    //GPIO3=input, GPIO[2:0]=output
    SSD1963_data8(0x01);    //GPIO0 normal

    SSD1963_command(SSD1963_SetPixelDataInterface);
#ifdef DATA_BUS_24
      SSD1963_data8(0x00);     // bah questo è 8bit ossia credo 3 colpi da 8
#else
      SSD1963_data8(0x03);
#endif

    __delay_ms(5);

    SSD1963_command(SSD1963_SetDisplayOn);
    SSD1963_command(SSD1963_SetDBCConf);
    SSD1963_data8(0x0d);
	}

void setAddrWindow(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UGRAPH_COORD_T h) {
  int x0a, x1a, y0a, y1a;

  switch(rotation) {
    case 0:
      w = x+w-1;
      h = y+h-1;
      break;
// VERIFICARE!! sugli altri non c'è rotation qua...      
    case 1:
      x0a = x;
      x1a = w;
      y0a = y;
      y1a = h;

      x = y0a;
      w = y1a;
      y = _width - 1 - x1a;
      h = _width - 1 - x0a;
      break;
      
    case 2:
      x0a = x;
      x1a = w;
      y0a = y;
      y1a = h;

      x = _width - 1 - x1a;
      w = _width - 1 - x0a;
      y = _height - 1 - y1a;
      h = _height - 1 - y0a;
      break;

    case 3:
      x0a = x;
      x1a = w;
      y0a = y;
      y1a = h;

      x = _height - 1 - y1a;
      w = _height - 1 - y0a;
      y = x0a;
      h = x1a;
      break;
    }

#if TFT_SSD1963 != TFTSPI
  SSD1963_command(SSD1963_SetColumnAddress);
  SSD1963_data8(HIBYTE(x));
  SSD1963_data8(LOBYTE(x));
  SSD1963_data8(HIBYTE(w));
  SSD1963_data8(LOBYTE(w));
  SSD1963_command(SSD1963_SetPageAddress);
  SSD1963_data8(HIBYTE(y));
  SSD1963_data8(LOBYTE(y));
  SSD1963_data8(HIBYTE(h));
  SSD1963_data8(LOBYTE(h));
  SSD1963_command(SSD1963_WriteMemoryStart);
#else
  x += _xstart;
  y += _ystart;
  uint32_t xa = ((uint32_t)x << 16) | (x+w-1);
  uint32_t ya = ((uint32_t)y << 16) | (y+h-1); 
  
  writeCommandCS(SSD1963_SetColumnAddress); // Column addr set
  writeData32(xa);

  writeCommandCS(SSD1963_SetPageAddress); // Row addr set
  writeData32(ya);

  writeCommandCS(SSD1963_WriteMemoryStart); // write to RAM
#endif
	}

void drawPixel(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UINT16 color) {

  //cliptoscreen
	if(boundaryCheck(x,y)) 
		return;

  setAddrWindow(x,y,1,1);
#if TFT_SSD1963 == TFTBUS18
  uint8_t red = (color & 0xF800) >> 8;
  red |= ((red & 0x08) ? 0x07 : 0x00);
  SSD1963_data8(red);
  uint8_t green = (color & 0x7E0) >> 3;
  green |= ((green & 0x08) ? 0x07 : 0x00);
  SSD1963_data8(green);
  uint8_t blue = (color & 0x1F) << 3;
  blue |= ((blue & 0x08) ? 0x07 : 0x00);
  SSD1963_data8(blue);
#elif TFT_SSD1963 == TFTBUS16
  SSD1963_data(color);
#elif TFT_SSD1963 == TFTBUS24
  SSD1963_data(color);
//  SSD1963_data24(color);
#else
  write24BitColor(color);
#endif
  }

void SSD1963_setPixel(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UINT16 color) {

  //cliptoscreen
  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height))
    return;

  setAddrWindow(x,y,1,1);
#if TFT_SSD1963 == TFTBUS18
  uint8_t red = (color & 0xF800) >> 8;
  red |= ((red & 0x08) ? 0x07 : 0x00);
  SSD1963_data8(red);
  uint8_t green = (color & 0x7E0) >> 3;
  green |= ((green & 0x08) ? 0x07 : 0x00);
  SSD1963_data8(green);
  uint8_t blue = (color & 0x1F) << 3;
  blue |= ((blue & 0x08) ? 0x07 : 0x00);
  SSD1963_data8(blue);
#elif TFT_SSD1963 == TFTBUS24
  SSD1963_data(color);
#else
  write24BitColor(color);
#endif
  }

void write24BitColor(UINT32 color) {
//ri-provare..
  m_LCDRSBit=1;
  m_LCDCSBit=0;
  SSD1963_data8(LOBYTE(LOWORD(color)));
  SSD1963_data8(HIBYTE(LOWORD(color)));
  SSD1963_data8(LOBYTE(HIWORD(color)));
  m_LCDCSBit=1;

	}

void fillScreen(UINT16 color) {

  fillRectangle(0, 0,  _width, _height, color);
	}

uint8_t SSD1963_clipToScreen(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UGRAPH_COORD_T h) {
  
  if((x < 0) || (x >= _width) || (h <= 0)) {
    return 0;
    }

  if(y < 0) {
    h += y;
    y = 0;
    if(h <= 0) {
      return 0;
      }
    }

  if(y >= _height) {
    return 0;
    }

  if(y + h >= _height) {
    h = _height-y;
    if(h <= 0) {
      return 0;
      }
    }

  if((x < 0) || (x >= _width) || (h <= 0)) {
    return 0;
    }

  if(y < 0) {
    h += y;
    y = 0;
    if(h <= 0) {
      return 0;
      }
    }

  if(y >= _height) {
    return 0;
    }

  if(y + h >= _height) {
    h = _height-y;
    if(h <= 0) {
      return 0;
      }
    }
  
  return 1;     //boh fare volendo (dovrebbe esserci in ADAfruit ma...
  }

void drawHorizontalLine(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UINT16 color) {

  //cliptoscreen
  
  if((y < 0) || (y >= _height) || (w <= 0)) {
    return;
    }

  if(x < 0) {
    w += x;
    x = 0;
    if (w <= 0) {
      return;
      }
    }

  if(x >= _width) {
    return;
    }

  if (x + w >= _width) {
    w = _width-x;
    if(w <= 0) {
      return;
      }
    }

  setAddrWindow(x, y, w, 1);
  while(w--) {
#if TFT_SSD1963 == TFTBUS18
    uint8_t red = (color & 0xF800) >> 8;
    red |= ((red & 0x08) ? 0x07 : 0x00);
    SSD1963_data8(red);
    uint8_t green = (color & 0x7E0) >> 3;
    green |= ((green & 0x08) ? 0x07 : 0x00);
    SSD1963_data8(green);
    uint8_t blue = (color & 0x1F) << 3;
    blue |= ((blue & 0x08) ? 0x07 : 0x00);
    SSD1963_data8(blue);
#elif TFT_SSD1963 == TFTBUS24
    SSD1963_data(color);
#else
    write24BitColor(color);
#endif
    }
	}

void drawVerticalLine(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T h, UINT16 color) {

  // cliptoscreen
  if((x < 0) || (x >= _width) || (h <= 0)) {
    return;
    }

  if(y < 0) {
    h += y;
    y = 0;
    if(h <= 0) {
      return;
      }
    }

  if(y >= _height) {
    return;
    }

  if(y + h >= _height) {
    h = _height-y;
    if(h <= 0) {
      return;
      }
    }

  setAddrWindow(x, y, 1, h);
  while(h--) {
#ifdef DATA_BUS_24
    uint8_t red = (color & 0xF800) >> 8;
    red |= ((red & 0x08) ? 0x07 : 0x00);
    SSD1963_data8(red);
    uint8_t green = (color & 0x7E0) >> 3;
    green |= ((green & 0x08) ? 0x07 : 0x00);
    SSD1963_data8(green);
    uint8_t blue = (color & 0x1F) << 3;
    blue |= ((blue & 0x08) ? 0x07 : 0x00);
    SSD1963_data8(blue);
#else
    SSD1963_data(color);
#endif
		}
	}

void drawFastVLine(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T h, UINT16 color) {

	// Rudimentary clipping
	if(boundaryCheck(x,y))
		return;
	if(((y + h) - 1) >= _height) 
		h = _height-y;
//	setAddr(x,y,x,(y+h)-1);
	VLine(x, y, h, color);
	}

void __attribute__((always_inline)) HLine(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UINT16 color) {
// NON passare 0 :)
	setAddrWindow(x, y, w, 1);
  m_LCDRSBit = 1;
  m_LCDCSBit = 0;
	do { SSD1963_data_raw(color); } while (--w > 0);
  m_LCDCSBit = 1;
	}

void __attribute__((always_inline)) VLine(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T h, UINT16 color) {
// NON passare 0 :)
	setAddrWindow(x, y, 1, h);
  m_LCDRSBit = 1;
  m_LCDCSBit = 0;
	do { SSD1963_data_raw(color); } while (--h > 0);
  m_LCDCSBit = 1;
	}

BOOL boundaryCheck(UGRAPH_COORD_T x,UGRAPH_COORD_T y) {

	if((x >= _width) || (y >= _height)) 
		return TRUE;
	return FALSE;
	}

void drawFastHLine(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UINT16 color) {

	// Rudimentary clipping
	if(boundaryCheck(x,y)) 
		return;
	if(((x+w) - 1) >= _width)  
		w = _width-x;
//	setAddr(x,y,(x+w)-1,y);
	HLine(x, y, w, color);
	}

void drawLine(UGRAPH_COORD_T x0, UGRAPH_COORD_T y0,UGRAPH_COORD_T x1, UGRAPH_COORD_T y1, UINT16 color) {
	BOOL steep;
	GRAPH_COORD_T dx,dy;
	GRAPH_COORD_T err;
	GRAPH_COORD_T ystep;
	GRAPH_COORD_T xbegin;

	if(y0==y1) {
		if(x1>x0) {
			drawFastHLine(x0, y0, x1-x0+1, color);
			} 
		else if(x1 < x0) {
			drawFastHLine(x1, y0, x0-x1+1, color);
			} 
		else {
			drawPixel(x0, y0, color);
			}
		return;
		} 
	else if(x0==x1) {
		if(y1>y0) {
			drawFastVLine(x0, y0, y1-y0+1, color);
			} 
		else {
			drawFastVLine(x0, y1, y0-y1+1, color);
			}
		return;
		}

	steep = abs(y1-y0) > abs(x1-x0);
	if(steep) {
		_swap(&x0, &y0);
		_swap(&x1, &y1);
		}
	if(x0>x1) {
		_swap(&x0, &x1);
		_swap(&y0, &y1);
		}

	dx = x1-x0;
	dy = abs(y1-y0);

	err = dx/2;

	if(y0<y1) {
		ystep = 1;
		} 
	else {
		ystep = -1;
		}

	
	xbegin = x0;
	if(steep) {
		for(; x0<=x1; x0++) {
			err -= dy;
			if(err < 0) {
				INT16 len = x0-xbegin;
				if(len) {
					VLine(y0, xbegin, len + 1, color);
					} 
				else {
					drawPixel(y0, x0, color);
					}
				xbegin = x0+1;
				y0 += ystep;
				err += dx;
				}
			}
		if (x0 > xbegin + 1) {
			VLine(y0, xbegin, x0 - xbegin, color);
			}
		} 
	else {
		for(; x0<=x1; x0++) {
			err -= dy;
			if(err < 0) {
				INT16 len = x0-xbegin;
				if(len) {
					HLine(xbegin, y0, len+1, color);
					} 
				else {
					drawPixel(x0, y0, color);
					}
				xbegin = x0+1;
				y0 += ystep;
				err += dx;
				}
			}
		if(x0 > xbegin+1) {
			HLine(xbegin, y0, x0-xbegin, color);
			}
		}
//	writecommand(CMD_NOP);
	}

void fillRectangle(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UGRAPH_COORD_T h, UINT16 color) {
  int i;

  if(!SSD1963_clipToScreen(x, y, w, h))
    return;

#ifndef USING_SIMULATOR
  setAddrWindow(x, y, w, h);
  m_LCDCSBit = 0;
  m_LCDRSBit = 1;
  for(i=0; i < w * h; i++) {
#if TFT_SSD1963 == TFTBUS18
    uint8_t red = (color & 0xF800) >> 8;
    red |= ((red & 0x08) ? 0x07 : 0x00);
    SSD1963_data8(red);
    uint8_t green = (color & 0x7E0) >> 3;
    green |= ((green & 0x08) ? 0x07 : 0x00);
    SSD1963_data8(green);
    uint8_t blue = (color & 0x1F) << 3;
    blue |= ((blue & 0x08) ? 0x07 : 0x00);
    SSD1963_data8(blue);
#elif TFT_SSD1963 == TFTBUS24
    SSD1963_data_raw(color);
#else
    write24BitColor(color);
#endif
    }
  m_LCDCSBit = 1;
#endif
  }

// fill a rectangle
void fillRect(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UGRAPH_COORD_T h, UINT16 color) {

	if(boundaryCheck(x,y)) 
		return;
	if(((x + w) - 1) >= _width)  
		w = _width  - x;
	if(((y + h) - 1) >= _height) 
		h = _height - y;
	setAddrWindow(x,y,w,h);
  m_LCDCSBit = 0;
  m_LCDRSBit = 1;
	for(y=h; y>0; y--) {
		for(x=w; x>1; x--) {
			SSD1963_data_raw(color);
			}
		SSD1963_data_raw(color);
		}
  m_LCDCSBit = 1;
	}

void drawRect(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UGRAPH_COORD_T h, UINT16 color){

	HLine(x, y, w, color);
	HLine(x, y+h-1, w, color);
	VLine(x, y, h, color);
	VLine(x+w-1, y, h, color);
	}


void SSD1963_invertDisplay(unsigned char i) {

  SSD1963_command(i ? SSD1963_EnterInvertMode : SSD1963_ExitInvertMode);
	}

void SSD1963_displayOn(void) {

  SSD1963_command(SSD1963_ExitIdleMode);
  SSD1963_command(SSD1963_SetDisplayOn);
  SSD1963_command(SSD1963_EnterSleepMode);
  SSD1963_enableBacklight();
	}

void SSD1963_displayOff(void) {

  SSD1963_command(SSD1963_ExitSleepMode);
  SSD1963_command(SSD1963_SetDisplayOff);
  SSD1963_command(SSD1963_EnterIdleMode);
  SSD1963_disableBacklight();
	}


void SSD1963_windowData(UINT16 d) {

  SSD1963_data(d);
	}

void SSD1963_windowData2(const UINT16 *d, int l) {
	int i;

  for(i=0; i < l; i++) {
    SSD1963_data(d[i]);
    }
	}

void SSD1963_openWindow(UGRAPH_COORD_T x0, UGRAPH_COORD_T y0, UGRAPH_COORD_T x1, UGRAPH_COORD_T y1) {

  setAddrWindow(x0, y0, x1-x0+1, y1-y0+1);
	}

void SSD1963_enableBacklight(void) {

  SSD1963_command(SSD1963_SetPWMConf);
  SSD1963_data8(0x06);
  SSD1963_data8(0xFF);
  SSD1963_data8(0x01);
  SSD1963_data8(0xFF);
  SSD1963_data8(0x00);
  SSD1963_data8(0x00);
  _brightness = 0xff;
	}

void SSD1963_disableBacklight(void) {

  SSD1963_command(SSD1963_SetPWMConf);
  SSD1963_data8(0x06);
  SSD1963_data8(0xFF);
  SSD1963_data8(0x00);
  SSD1963_data8(_brightness);
  SSD1963_data8(0x00);
  SSD1963_data8(0x00);
	}

void SSD1963_setBacklight(uint8_t b) {

  _brightness = b;
  SSD1963_command(SSD1963_SetPWMConf);
  SSD1963_data8(0x06);
  SSD1963_data8(_brightness);
  SSD1963_data8(0x01);
  SSD1963_data8(_brightness);
  SSD1963_data8(0x00);
  SSD1963_data8(0x00);
	}

UINT16 SSD1963_colorAt(UGRAPH_COORD_T x, UGRAPH_COORD_T y) {

  setAddrWindow(x, y, 1, 1);
  SSD1963_command(0x2E);
  return SSD1963_read();
	}


/*------------------------- PORT B VARIANT ------------------------------*/
#if 0
void SSD1963_PORTB_SSD1963_PORTB(uint8_t tft_bus_width) {

  bus_width = tft_bus_width;
  _profile=TFT7;
	}

void SSD1963_PORTB_command(uint16_t cmd) {

  TRISB = 0x0000;
  m_LCDRSBit = 0;
  m_LCDCSBit = 0;
  LATB = cmd;
#ifdef m_LCDWRBit
  m_LCDWRBit=0;
  DELAY_LCD();
  m_LCDWRBit=1;
#endif
  m_LCDCSBit = 1;
	}

void SSD1963_PORTB_data8(uint8_t cmd) {

  TRISB = 0x0000;
  m_LCDRSBit = 1;
  m_LCDCSBit = 0;
  LATB = cmd;
#ifdef m_LCDWRBit
  m_LCDWRBit=0;
  DELAY_LCD();
  m_LCDWRBit=1;
#endif
  m_LCDCSBit = 1;
	}

void SSD1963_PORTB_data(uint16_t cmd) {

  TRISB = 0x0000;
  m_LCDRSBit = 1;
  m_LCDCSBit = 0;
  LATB = cmd;
#ifdef m_LCDWRBit
  m_LCDWRBit=0;
  DELAY_LCD();
  m_LCDWRBit=1;
#endif
  m_LCDCSBit = 1;
	}

uint16_t SSD1963_PORTB_read(void) {

  TRISB = 0xFFFF;
  m_LCDRSBit = 1;
  DELAY_LCD();
#ifdef m_LCDRDBit
  m_LCDRDBit=0;
#endif
  DELAY_LCD();
  m_LCDCSBit = 0;
  DELAY_LCD();
  uint16_t out = PORTB;
#ifdef m_LCDRDBit
  m_LCDRDBit=1;
#endif
  TRISB = 0x0000;
  m_LCDCSBit = 1;
  return out;
	}

void SSD1963_PORTB_initInterface(void) {

  TRISFbits.TRISF3=0;
  TRISFbits.TRISF4=0;
#ifdef m_LCDRDBit
  TRISFbits.TRISF0=0;
  TRISFbits.TRISF1=0;
#endif
#ifdef m_LCDResBit
  pinMode(pin_reset, OUTPUT);
#endif

  ANSELB = 0x0000;
//  ANSELF = 0x0000;

  m_LCDRSBit=1;
  m_LCDCSBit=1;
#ifdef m_LCDRDBit
  m_LCDRDBit=1;
  m_LCDWRBit=1;
#endif
  
  TRISB = 0x0000;

#ifdef m_LCDResBit
  pinMode(pin_reset, OUTPUT);
  digitalWrite(pin_reset, 1);
  __delay_ms(100);
  digitalWrite(pin_reset, 0);
  __delay_ms(100);
  digitalWrite(pin_reset, 1);
#endif
  __delay_ms(100);
	}

void SSD1963_PORTB_openWindow(int x0, int y0, int x1, int y1) {

  setAddrWindow(x0, y0, x1-x0+1, y1-y0+1);
  m_LCDCSBit=0;
  m_LCDRSBit = 1;
	}

void SSD1963_PORTB_windowData(UINT16 d) {

  LATB = d;
#ifdef m_LCDWRBit
  m_LCDWRBit=0;
  DELAY_LCD();
  m_LCDWRBit=1;
#endif
	}

void SSD1963_PORTB_windowData2(UINT16 *d, int l) {
	int x;

  for(x=0; x < l; x++) {
    LATB = d[x];
#ifdef m_LCDWRBit
    m_LCDWRBit=0;
  DELAY_LCD();
    m_LCDWRBit=1;
#endif
    }
	}

void SSD1963_PORTB_closeWindow(void) {

  m_LCDCSBit = 1;
	}
#endif 


void _swap(UGRAPH_COORD_T *a, UGRAPH_COORD_T *b) {
	UGRAPH_COORD_T t = *a; 
	*a = *b; 
	*b = t; 
	}

void setRotation(uint8_t m) {    // 
  uint8_t madctl = 0;

  rotation = m & 3; // can't be higher than 3
// FINIRE!!!
  switch(rotation) {
    case 0:
      _width  = SSD1963_TFTWIDTH;
      _height = SSD1963_TFTHEIGHT;
      _xstart   = 0;
      _ystart   = 0;
      break;
     
   case 1:
      _width  = SSD1963_TFTHEIGHT;
      _height = SSD1963_TFTWIDTH;
      _xstart   = 0;
      _ystart   = 0;
      break;
     
  case 2:
      _width  = SSD1963_TFTWIDTH;
      _height = SSD1963_TFTHEIGHT;
      _xstart   = 0;
      _ystart   = 0;
      break;
     
   case 3:
      _width  = SSD1963_TFTHEIGHT;
      _height = SSD1963_TFTWIDTH;
      _xstart   = 0;
      _ystart   = 0;
      break;
    }

//  sendCommand(ST77XX_MADCTL, &madctl, 1);
  }

void scrollArea(UGRAPH_COORD_T x1, UGRAPH_COORD_T y1, UGRAPH_COORD_T x2, UGRAPH_COORD_T y2, int16_t dx, int16_t dy) {
  int x,y;
  WORD buffer[_width /* *dy */];   // beh, occhio...
  
  if(dy<0) {
    for(y=y1; y<=y2; y++) {
      WORD *p1;
      SSD1963_command(SSD1963_SetColumnAddress);
      SSD1963_data8(HIBYTE(x1));
      SSD1963_data8(LOBYTE(x1));
      SSD1963_data8(HIBYTE(x2));
      SSD1963_data8(LOBYTE(x2));
      SSD1963_command(SSD1963_SetPageAddress);
      SSD1963_data8(HIBYTE(y));
      SSD1963_data8(LOBYTE(y));
      SSD1963_data8(HIBYTE(y));
      SSD1963_data8(LOBYTE(y));
      SSD1963_command(SSD1963_ReadMemoryStart);
      p1=buffer;
      // per ora solo dy e su righe intere...
      // e ovviamente occhio alle sovrascritture!
      for(x=x1; x<x2; x++) {
        *(p1+x)=SSD1963_read();
        }
      SSD1963_command(SSD1963_SetColumnAddress);
      SSD1963_data8(HIBYTE(x1+dx));
      SSD1963_data8(LOBYTE(x1+dx));
      SSD1963_data8(HIBYTE(x2+dx));
      SSD1963_data8(LOBYTE(x2+dx));
      SSD1963_command(SSD1963_SetPageAddress);
      SSD1963_data8(HIBYTE(y+dy));
      SSD1963_data8(LOBYTE(y+dy));
      SSD1963_data8(HIBYTE(y+dy));
      SSD1963_data8(LOBYTE(y+dy));
      SSD1963_command(SSD1963_WriteMemoryStart);
      p1=buffer;
      m_LCDRSBit = 1;
      m_LCDCSBit = 0;
      for(x=x1; x<x2; x++) {
        SSD1963_data_raw(*(p1+x));
        }
      m_LCDCSBit = 1;
      ClrWdt();
      }
    for(y=y2+dy; y<=y2; y++) {// PULISCO COSì sempre! o fillrect... o HLine
      m_LCDRSBit = 1;
      m_LCDCSBit = 0;
      for(x=x1; x<=x2; x++) {
        SSD1963_data_raw(textbgcolor);
        }
      m_LCDCSBit = 1;
      ClrWdt();
      }
    }
  else {
    for(y=y2; y>=y1; y--) {
      WORD *p1;
      SSD1963_command(SSD1963_SetColumnAddress);
      SSD1963_data8(HIBYTE(x1));
      SSD1963_data8(LOBYTE(x1));
      SSD1963_data8(HIBYTE(x2));
      SSD1963_data8(LOBYTE(x2));
      SSD1963_command(SSD1963_SetPageAddress);
      SSD1963_data8(HIBYTE(y));
      SSD1963_data8(LOBYTE(y));
      SSD1963_data8(HIBYTE(y));
      SSD1963_data8(LOBYTE(y));
      SSD1963_command(SSD1963_ReadMemoryStart);
      p1=buffer;
      // per ora solo dy e su righe intere...
      // e ovviamente occhio alle sovrascritture!
      for(x=x1; x<=x2; x++) {
        *(p1+x)=SSD1963_read();
        }
      SSD1963_command(SSD1963_SetColumnAddress);
      SSD1963_data8(HIBYTE(x1+dx));
      SSD1963_data8(LOBYTE(x1+dx));
      SSD1963_data8(HIBYTE(x2+dx));
      SSD1963_data8(LOBYTE(x2+dx));
      SSD1963_command(SSD1963_SetPageAddress);
      SSD1963_data8(HIBYTE(y+dy));
      SSD1963_data8(LOBYTE(y+dy));
      SSD1963_data8(HIBYTE(y+dy));
      SSD1963_data8(LOBYTE(y+dy));
      SSD1963_command(SSD1963_WriteMemoryStart);
      p1=buffer;
      m_LCDRSBit = 1;
      m_LCDCSBit = 0;
      for(x=x1; x<=x2; x++) {
        SSD1963_data_raw(*(p1+x));
        }
      m_LCDCSBit = 1;
      ClrWdt();
      }
    for(y=y1; y<=y1+dy; y++) {// PULISCO COSì sempre! o fillrect... o HLine
      m_LCDRSBit = 1;
      m_LCDCSBit = 0;
      for(x=x1; x<=x2; x++) {
        SSD1963_data_raw(textbgcolor);
        }
      m_LCDCSBit = 1;
      ClrWdt();
      }
    }
  
  }

