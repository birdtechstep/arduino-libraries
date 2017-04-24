/* ------------------------------------------------------------------------- *
   @file              : Ayarafun_ST7789H2.cpp
   @author            : BIRD TECHSTEP [t.artsamart@gmail.com]
   @license           : BSD license, all text above and below must be
                        included in any redistribution.
   @description       : This is the library for TFT displays.
   Model Number       : SF-TC122B-8171B-N.
   Type               : TFT.
   Display & LCD Type : 240* RGB *204.
   Outline Dimemsions : 33.66*33.02*1.5mm.
   Display Size       : 1.22 inch.
   Driver             : ST7789H2

   @section           : HISTORY
    
   @version           : 1.0 - First release
   @date              : 03/25/2017 [mm/dd/yyyy]
 *-------------------------------------------------------------------------- */
#include "Ayarafun_ST7789H2.h"
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

// Many (but maybe not all) non-AVR board installs define macros
// for compatibility with existing PROGMEM-reading AVR code.
// Do our own checks and defines here for good measure...

#ifndef pgm_read_byte
 #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif
#ifndef pgm_read_word
 #define pgm_read_word(addr) (*(const unsigned short *)(addr))
#endif
#ifndef pgm_read_dword
 #define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#endif

// Pointers are a peculiar case...typically 16-bit on AVR boards,
// 32 bits elsewhere.  Try to accommodate both...

#if !defined(__INT_MAX__) || (__INT_MAX__ > 0xFFFF)
 #define pgm_read_pointer(addr) ((void *)pgm_read_dword(addr))
#else
 #define pgm_read_pointer(addr) ((void *)pgm_read_word(addr))
#endif

#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif

#if !defined(ST7789H2_USE_HWSPI_ONLY)
// Constructor when using software SPI.  All output pins are configurable.
Ayarafun_ST7789H2::Ayarafun_ST7789H2(int8_t cs, int8_t rs, int8_t sid, int8_t sclk, int8_t bl, int8_t rst) 
{
  _cs   = cs;
  _rs   = rs;
  _miso = -1;
  _sid  = sid;
  _sclk = sclk;
  _rst  = rst;
  _bl   = bl;
  hwSPI = false;
  // ------------
  _width    = ST7789H2_TFTWIDTH;
  _height   = ST7789H2_TFTHEIGHT;
  rotation  = 0;
  cursor_y  = cursor_x    = 0;
  textsize  = 1;
  textcolor = textbgcolor = 0xFFFF;
  wrap      = true;
  _cp437    = false;
  gfxFont   = NULL;
}
#endif

// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
Ayarafun_ST7789H2::Ayarafun_ST7789H2(int8_t cs, int8_t rs, int8_t bl, int8_t rst) {
  _cs   = cs;
  _rs   = rs;
  _miso = -1;
  _rst  = rst;
  _bl   = bl;
  hwSPI = true;
  _sid  = _sclk = 0;
  if (_cs == 15) {
    _sid  = 13;
    _sclk = 14;
  } else {
    _sid  = 23;
	_sclk = 18;
  }
  // ---------------
  _width    = ST7789H2_TFTWIDTH;
  _height   = ST7789H2_TFTHEIGHT;
  rotation  = 0;
  cursor_y  = cursor_x    = 0;
  textsize  = 1;
  textcolor = textbgcolor = 0xFFFF;
  wrap      = true;
  _cp437    = false;
  gfxFont   = NULL;
}

inline void Ayarafun_ST7789H2::spiwrite(uint8_t c) {
  if (hwSPI) {
	SPI.transfer(c);
  } else {
	  // Fast SPI bitbang swiped from LPD8806 library
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) {
	//digitalWrite(_mosi, HIGH); 
	*sidport |=  sidpinmask;
      } else {
	//digitalWrite(_mosi, LOW); 
	*sidport &= ~sidpinmask;
      }
      //digitalWrite(_sclk, HIGH);
      *clkport |=  clkpinmask;
      //digitalWrite(_sclk, LOW);
      *clkport &= ~clkpinmask;
    }
  //for(uint8_t bit = 0x80; bit; bit >>= 1) {
  //    if(c & bit) {
//	digitalWrite(_sid, HIGH); 
 //     } else {
//	digitalWrite(_sid, LOW); 
  //    }
    //  digitalWrite(_sclk, HIGH);
      //digitalWrite(_sclk, LOW);
    //}
  }
}
	
inline void Ayarafun_ST7789H2::spiwrite16(uint16_t v) {
  spiwrite((uint8_t)(v >> 8));
  spiwrite((uint8_t)(v >> 0));
}

inline void Ayarafun_ST7789H2::setCS(bool level) {
  digitalWrite(_cs, level);
}

inline void Ayarafun_ST7789H2::setRS(bool level) {
  digitalWrite(_rs, level);
}

void Ayarafun_ST7789H2::writecommand(uint8_t c) {
  setRS(false);
  setCS(false);

  spiwrite(c);

  setCS(true);
}


void Ayarafun_ST7789H2::writedata(uint8_t c) {
  setRS(true);
  setCS(false);
    
  spiwrite(c);

  setCS(true);
} 

void Ayarafun_ST7789H2::writedata16(uint16_t c) {
  setRS(true);
  setCS(false);
    
  spiwrite16(c);

  setCS(true);
} 

#define DELAY 0x80
static const uint8_t PROGMEM Bcmd[] = {
	21,                           // 21  Commands in list:
	ST7789H2_SWRESET  , DELAY,    //  1: [0x01] Software Reset
	250,
	ST7789H2_SLPOUT   , DELAY,    //  2: [0x11] Sleep Out
	120,
	ST7789H2_MADCTL   ,  1   ,    //  3: [0x36] Memory Data Access Control
	0x00,
	ST7789H2_COLMOD   ,  1   ,    //  4: [0x3A] Interface Pixel Format
	0x55,
	ST7789H2_INVON    ,  0   ,    //  5: [0x21] Display Inversion On
	ST7789H2_PORCTRL  ,  5   ,    //  6: [0xB2] Porch Setting
	0x0C,
	0x0C,
	0x00,
	0x33,
	0x33,
	ST7789H2_GCTRL    ,  1   ,    //  7: [0xB7] Gate Control
	0x35,
	ST7789H2_VCOMS    ,  1   ,    //  8: [0xBB] VCOM Setting
	0x2B,
	ST7789H2_LCMCTRL  ,  1   ,    //  9: [0xC0] LCM Control
	0x2C,
	ST7789H2_VDVVRHEN ,  1   ,    // 10: [0xC2] VDV and VRH Command Enable
	0x01,
	ST7789H2_VRHS     ,  1   ,    // 11: [0xC3] VRH Set
	0x11,
	ST7789H2_VDVS     ,  1   ,    // 12: [0xC4] VDV Set
	0x20,
	ST7789H2_FRCTRL2  ,  1   ,    // 13: [0xC6] Frame Rate Control in Normal Mode
	0x0F,
	ST7789H2_PWCTRL1  ,  2   ,    // 14: [0xD0] Power Control 1
	0xA4,
	0xA1,
	ST7789H2_PVGAMCTRL, 14   ,    // 15: [0xE0] Positive Voltage Gamma Control
    0xD0,
    0x00,
    0x05,
    0x0E,
    0x15,
    0x0D,
    0x37,
    0X43,
    0x47,
    0x09,
    0x15,
    0x12,
    0x16,
    0x19,
	ST7789H2_NVGAMCTRL, 14   ,    // 16: [0xE1] Negative Voltage Gamma Control
    0xD0,
    0x00,
    0x05,
    0x0D,
    0x0C,
    0x06,
    0x2D,
    0x44,
    0x40,
    0x0E,
    0x1C,
    0x18,
    0x16,
    0x19,
	ST7789H2_RGBCTRL  ,  3   ,    // 17: [0xB1] RGB Interface Control
    0x00,
    0x04,
    0x14,
	ST7789H2_CASET    ,  4   ,    // 18: [0x2A] Column Address Set
    0x00,
    0x00,
    0x00,
    0xEF,  // 239
	ST7789H2_RASET    ,  4   ,    // 19: [0x2B] Row Address Set
    0x00,
    0x00,
    0x00,
    0xEF,  // 239 0xCB : 203
	ST7789H2_DISPON   , DELAY,    // 20: [0x29] Display On
    20,
	ST7789H2_RAMWR    , 0};       // 21: [0x2C] Memory Write


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void Ayarafun_ST7789H2::commandList(const uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    writecommand(pgm_read_byte(addr++)); //   Read, issue command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
      writedata(pgm_read_byte(addr++));  //     Read, issue argument
    }

    if(ms) {
      ms = pgm_read_byte(addr++);        // Read post-command delay time (ms)
      if(ms == 255) ms = 500;            // If 255, delay for 500 ms
      delay(ms);
    }
  }
}


void Ayarafun_ST7789H2::commonInit(const uint8_t *cmdList) {

  pinMode(_rs, OUTPUT);
  pinMode(_cs, OUTPUT);
  //pinMode(_bl, OUTPUT);
  
	
  if(hwSPI) { // Using hardware SPI
    //SPI.begin();
	SPI.begin(_sclk, _miso, _sid, _cs);
	//SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
  } else {
    pinMode(_sclk, OUTPUT);
    pinMode(_sid , OUTPUT);

    clkport     = portOutputRegister(digitalPinToPort(_sclk));
    clkpinmask  = digitalPinToBitMask(_sclk);
    sidport    = portOutputRegister(digitalPinToPort(_sid));
    sidpinmask = digitalPinToBitMask(_sid);
    *clkport   &= ~clkpinmask;
    *sidport  &= ~sidpinmask;

	}
  //digitalWrite(_bl, LOW);
  backlight(0);

  // toggle RST low to reset; CS low so it'll listen to us
  setCS(false);

  reset();
  //if(cmdList) commandList(cmdList);
  initial();
  
}

void Ayarafun_ST7789H2::reset(void) {
  if (_rst) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, HIGH);
    delay(500);
    digitalWrite(_rst, LOW);
    delay(500);
    digitalWrite(_rst, HIGH);
    delay(500);
  }	
}

// ---------------------------------------------------------------------------------
void Ayarafun_ST7789H2::initial(void) {
  writecommand(ST7789H2_SWRESET); // Software Reset
	delay(250);
  writecommand(0x11); // Sleep Out
    delay(120);
//-------------------------------- Display and color format setting ----------------------------//
  writecommand(0x36);  // Memory Data Access Control
    writedata(0x00);
	
  writecommand(0x3A);  // Interface Pixel Format
    writedata(0X05); //0x05
    //writedata(0x55);  // 65K of RGB interface : 16bit/pixel

//-------------------------------- ST7789V Frame rate setting ----------------------------------//
  writecommand(0xB2);  // Porch Setting
    writedata(0x0C); //0x0C
    writedata(0x0C); //0x0C
    writedata(0x00); //0x00
    writedata(0x33); //0x33
    writedata(0x33); //0x33
    
  writecommand(0xB7);  // Gate Control
    writedata(0x35);
//--------------------------------- ST7789V Power setting --------------------------------------//
  writecommand(0xBB);  // VCOM Setting
    writedata(0x2B); //0x35 0x1A
	
  writecommand(0xC0);  // LCM Control
    writedata(0x2C);
	
  writecommand(0xC2);  // VDV and VRH Command Enable
    writedata(0x01);
	
  writecommand(0xC3);  // VRH Set
    writedata(0x11);  //0x0B
	
  writecommand(0xC4);  // VDV Set
    writedata(0x20);
	
  writecommand(0xC6);  // Frame Rate Control in Normal Mode
    writedata(0x0F); //0x0F
	
  writecommand(0xD0);  // Power Control 1
    writedata(0xA4);
    writedata(0xA1);
	
  writecommand(0x21);    //0x21  // Display Inversion On
  
//-------------------------------- ST7789V gamma setting ---------------------------------------//
  writecommand(0xE0);  // Positive Voltage Gamma Control
	writedata(0xD0);
    writedata(0x00);
    writedata(0x05);
    writedata(0x0E);
    writedata(0x15);
    writedata(0x0D);
    writedata(0x37);
    writedata(0X43);
    writedata(0x47);
    writedata(0x09);
    writedata(0x15);
    writedata(0x12);
    writedata(0x16);
    writedata(0x19);
  writecommand(0xE1);  // Negative Voltage Gamma Control
	writedata(0xD0);
    writedata(0x00);
    writedata(0x05);
    writedata(0x0D);
    writedata(0x0C);
    writedata(0x06);
    writedata(0x2D);
    writedata(0x44);
    writedata(0x40);
    writedata(0x0E);
    writedata(0x1C);
    writedata(0x18);
    writedata(0x16);
    writedata(0x19);

  writecommand(0xB1);  // RGB Interface Control
    writedata(0x00);
    writedata(0x04);
    writedata(0x14);

  writecommand(0x2A);  // Column Address Set
    writedata(0x00);
    writedata(0x00);
    writedata(0x00);
    writedata(0xEF);  // 239

  writecommand(0x2B);  // Row Address Set
    writedata(0x00);
    writedata(0x00);
    writedata(0x00);
    writedata(0xEF);  // 239

  writecommand(0x29);  // Display On
    delay(20);
  writecommand(0x2C);  // Memory Write
    delay(200);
}
// ---------------------------------------------------------------------------------

// Initialization for ST7789H2 screens
void Ayarafun_ST7789H2::begin(void) {
  commonInit(Bcmd);
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  //SPI.setClockDivider(10); // 16 MHz
  //SPI.setBitOrder(MSBFIRST);
  //SPI.setDataMode(SPI_MODE0);
  //digitalWrite(_bl, HIGH);
  backlight(1);

}


void Ayarafun_ST7789H2::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {

  writecommand(ST7789H2_CASET); // Column addr set
  writedata16(x0);
  writedata16(x1);

  writecommand(ST7789H2_RASET); // Row addr set
  writedata16(y0);
  writedata16(y1);

  writecommand(ST7789H2_RAMWR); // write to RAM
}


void Ayarafun_ST7789H2::pushColor(uint16_t color) {
  setRS(true);
  setCS(false);
  
  spiwrite16(color);

  setCS(true);
}

void Ayarafun_ST7789H2::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;
  setAddrWindow(x,y,x+1,y+1);
  writedata16(color);
}

void Ayarafun_ST7789H2::drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((y+h-1) >= _height) h = _height-y;
  setAddrWindow(x, y, x, y+h-1);
  writeColor(color, h);
}


void Ayarafun_ST7789H2::drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  
  setAddrWindow(x, y, x+w-1, y);
  writeColor(color, w);
}



void Ayarafun_ST7789H2::fillScreen(uint16_t color) {
  fillRect(0, 0,  _width, _height, color);
}


void Ayarafun_ST7789H2::writeColor(uint16_t color, uint16_t count) {
  setRS(true);
  setCS(false);

  for (; count != 0; count--) {
    spiwrite16(color);
  }
  
  setCS(true);
}

// fill a rectangle
void Ayarafun_ST7789H2::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);
  writeColor(color, w*h);
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Ayarafun_ST7789H2::Color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void Ayarafun_ST7789H2::setRotation(uint8_t m) {

  writecommand(ST7789H2_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
  case 0:
     writedata(MADCTL_RGB);
     _width  = ST7789H2_TFTWIDTH;
     _height = ST7789H2_TFTHEIGHT;

    break;
   case 1:
     writedata(MADCTL_MX | MADCTL_MV | MADCTL_RGB);
     _width  = ST7789H2_TFTHEIGHT;
     _height = ST7789H2_TFTWIDTH;
     break;
   case 2:
     writedata(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
     _width  = ST7789H2_TFTWIDTH;
     _height = ST7789H2_TFTHEIGHT;

     break;
   case 3:
     writedata(MADCTL_MY | MADCTL_MV | MADCTL_RGB);
     _width  = ST7789H2_TFTHEIGHT;
     _height = ST7789H2_TFTWIDTH;
     break;
  }

}

void Ayarafun_ST7789H2::backlight(boolean i) {
  if (_bl) {
    pinMode(_bl, OUTPUT);
    digitalWrite(_bl, i);
  }	
}

void Ayarafun_ST7789H2::invertDisplay(boolean i) {
  writecommand(i ? ST7789H2_INVON : ST7789H2_INVOFF);
}


/* -------------------------------------------------------- */
// Draw a circle outline
void Ayarafun_ST7789H2::drawCircle(int16_t x0, int16_t y0, int16_t r,
 uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  drawPixel(x0  , y0+r, color);
  drawPixel(x0  , y0-r, color);
  drawPixel(x0+r, y0  , color);
  drawPixel(x0-r, y0  , color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    drawPixel(x0 + x, y0 + y, color);
    drawPixel(x0 - x, y0 + y, color);
    drawPixel(x0 + x, y0 - y, color);
    drawPixel(x0 - x, y0 - y, color);
    drawPixel(x0 + y, y0 + x, color);
    drawPixel(x0 - y, y0 + x, color);
    drawPixel(x0 + y, y0 - x, color);
    drawPixel(x0 - y, y0 - x, color);
  }
}

void Ayarafun_ST7789H2::drawCircleHelper( int16_t x0, int16_t y0,
 int16_t r, uint8_t cornername, uint16_t color) {
  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x4) {
      drawPixel(x0 + x, y0 + y, color);
      drawPixel(x0 + y, y0 + x, color);
    }
    if (cornername & 0x2) {
      drawPixel(x0 + x, y0 - y, color);
      drawPixel(x0 + y, y0 - x, color);
    }
    if (cornername & 0x8) {
      drawPixel(x0 - y, y0 + x, color);
      drawPixel(x0 - x, y0 + y, color);
    }
    if (cornername & 0x1) {
      drawPixel(x0 - y, y0 - x, color);
      drawPixel(x0 - x, y0 - y, color);
    }
  }
}

void Ayarafun_ST7789H2::fillCircle(int16_t x0, int16_t y0, int16_t r,
 uint16_t color) {
  drawFastVLine(x0, y0-r, 2*r+1, color);
  fillCircleHelper(x0, y0, r, 3, 0, color);
}

// Used to do circles and roundrects
void Ayarafun_ST7789H2::fillCircleHelper(int16_t x0, int16_t y0, int16_t r,
 uint8_t cornername, int16_t delta, uint16_t color) {

  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;

    if (cornername & 0x1) {
      drawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
      drawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
    }
    if (cornername & 0x2) {
      drawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
      drawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
    }
  }
}


// Draw a triangle
void Ayarafun_ST7789H2::drawTriangle(int16_t x0, int16_t y0,
 int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
  drawLine(x0, y0, x1, y1, color);
  drawLine(x1, y1, x2, y2, color);
  drawLine(x2, y2, x0, y0, color);
}

// Fill a triangle
void Ayarafun_ST7789H2::fillTriangle(int16_t x0, int16_t y0,
 int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {

  int16_t a, b, y, last;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (y0 > y1) {
    _swap_int16_t(y0, y1); _swap_int16_t(x0, x1);
  }
  if (y1 > y2) {
    _swap_int16_t(y2, y1); _swap_int16_t(x2, x1);
  }
  if (y0 > y1) {
    _swap_int16_t(y0, y1); _swap_int16_t(x0, x1);
  }

  if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
    a = b = x0;
    if(x1 < a)      a = x1;
    else if(x1 > b) b = x1;
    if(x2 < a)      a = x2;
    else if(x2 > b) b = x2;
    drawFastHLine(a, y0, b-a+1, color);
    return;
  }

  int16_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1;
  int32_t
    sa   = 0,
    sb   = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if(y1 == y2) last = y1;   // Include y1 scanline
  else         last = y1-1; // Skip it

  for(y=y0; y<=last; y++) {
    a   = x0 + sa / dy01;
    b   = x0 + sb / dy02;
    sa += dx01;
    sb += dx02;
    /* longhand:
    a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) _swap_int16_t(a,b);
    drawFastHLine(a, y, b-a+1, color);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);
  for(; y<=y2; y++) {
    a   = x1 + sa / dy12;
    b   = x0 + sb / dy02;
    sa += dx12;
    sb += dx02;
    /* longhand:
    a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) _swap_int16_t(a,b);
    drawFastHLine(a, y, b-a+1, color);
  }
}

// Bresenham's algorithm - thx wikpedia
void Ayarafun_ST7789H2::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
 uint16_t color) {
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    _swap_int16_t(x0, y0);
    _swap_int16_t(x1, y1);
  }

  if (x0 > x1) {
    _swap_int16_t(x0, x1);
    _swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++) {
    if (steep) {
      drawPixel(y0, x0, color);
    } else {
      drawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

// Draw a rectangle
void Ayarafun_ST7789H2::drawRect(int16_t x, int16_t y, int16_t w, int16_t h,
 uint16_t color) {
  drawFastHLine(x, y, w, color);
  drawFastHLine(x, y+h-1, w, color);
  drawFastVLine(x, y, h, color);
  drawFastVLine(x+w-1, y, h, color);
}


// Draw a rounded rectangle
void Ayarafun_ST7789H2::drawRoundRect(int16_t x, int16_t y, int16_t w,
 int16_t h, int16_t r, uint16_t color) {
  // smarter version
  drawFastHLine(x+r  , y    , w-2*r, color); // Top
  drawFastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
  drawFastVLine(x    , y+r  , h-2*r, color); // Left
  drawFastVLine(x+w-1, y+r  , h-2*r, color); // Right
  // draw four corners
  drawCircleHelper(x+r    , y+r    , r, 1, color);
  drawCircleHelper(x+w-r-1, y+r    , r, 2, color);
  drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
  drawCircleHelper(x+r    , y+h-r-1, r, 8, color);
}

// Fill a rounded rectangle
void Ayarafun_ST7789H2::fillRoundRect(int16_t x, int16_t y, int16_t w,
 int16_t h, int16_t r, uint16_t color) {
  // smarter version
  fillRect(x+r, y, w-2*r, h, color);

  // draw four corners
  fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
  fillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
}

// Draw a 1-bit image (bitmap) at the specified (x,y) position from the
// provided bitmap buffer (must be PROGMEM memory) using the specified
// foreground color (unset bits are transparent).
void Ayarafun_ST7789H2::drawBitmap(int16_t x, int16_t y,
 const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {

  int16_t i, j, byteWidth = (w + 7) / 8;
  uint8_t byte;

  for(j=0; j<h; j++) {
    for(i=0; i<w; i++) {
      if(i & 7) byte <<= 1;
      else      byte   = pgm_read_byte(bitmap + j * byteWidth + i / 8);
      if(byte & 0x80) drawPixel(x+i, y+j, color);
    }
  }
}

// Draw a 1-bit image (bitmap) at the specified (x,y) position from the
// provided bitmap buffer (must be PROGMEM memory) using the specified
// foreground (for set bits) and background (for clear bits) colors.
void Ayarafun_ST7789H2::drawBitmap(int16_t x, int16_t y,
 const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color, uint16_t bg) {

  int16_t i, j, byteWidth = (w + 7) / 8;
  uint8_t byte;

  for(j=0; j<h; j++) {
    for(i=0; i<w; i++ ) {
      if(i & 7) byte <<= 1;
      else      byte   = pgm_read_byte(bitmap + j * byteWidth + i / 8);
      if(byte & 0x80) drawPixel(x+i, y+j, color);
      else            drawPixel(x+i, y+j, bg);
    }
  }
}

// drawBitmap() variant for RAM-resident (not PROGMEM) bitmaps.
void Ayarafun_ST7789H2::drawBitmap(int16_t x, int16_t y,
 uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {

  int16_t i, j, byteWidth = (w + 7) / 8;
  uint8_t byte;

  for(j=0; j<h; j++) {
    for(i=0; i<w; i++ ) {
      if(i & 7) byte <<= 1;
      else      byte   = bitmap[j * byteWidth + i / 8];
      if(byte & 0x80) drawPixel(x+i, y+j, color);
    }
  }
}

// drawBitmap() variant w/background for RAM-resident (not PROGMEM) bitmaps.
void Ayarafun_ST7789H2::drawBitmap(int16_t x, int16_t y,
 uint8_t *bitmap, int16_t w, int16_t h, uint16_t color, uint16_t bg) {

  int16_t i, j, byteWidth = (w + 7) / 8;
  uint8_t byte;

  for(j=0; j<h; j++) {
    for(i=0; i<w; i++ ) {
      if(i & 7) byte <<= 1;
      else      byte   = bitmap[j * byteWidth + i / 8];
      if(byte & 0x80) drawPixel(x+i, y+j, color);
      else            drawPixel(x+i, y+j, bg);
    }
  }
}

//Draw XBitMap Files (*.xbm), exported from GIMP,
//Usage: Export from GIMP to *.xbm, rename *.xbm to *.c and open in editor.
//C Array can be directly used with this function
void Ayarafun_ST7789H2::drawXBitmap(int16_t x, int16_t y,
 const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {

  int16_t i, j, byteWidth = (w + 7) / 8;
  uint8_t byte;

  for(j=0; j<h; j++) {
    for(i=0; i<w; i++ ) {
      if(i & 7) byte >>= 1;
      else      byte   = pgm_read_byte(bitmap + j * byteWidth + i / 8);
      if(byte & 0x01) drawPixel(x+i, y+j, color);
    }
  }
}

size_t Ayarafun_ST7789H2::write(uint8_t c) {
	
  if(!gfxFont) { // 'Classic' built-in font

    if(c == '\n') {
      cursor_y += textsize*8;
      cursor_x  = 0;
    } else if(c == '\r') {
      // skip em
    } else {
      if(wrap && ((cursor_x + textsize * 6) >= _width)) { // Heading off edge?
        cursor_x  = 0;            // Reset x to zero
        cursor_y += textsize * 8; // Advance y one line
      }
      drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
      cursor_x += textsize * 6;
    }

  } else { // Custom font

    if(c == '\n') {
      cursor_x  = 0;
      cursor_y += (int16_t)textsize *
                  (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
    } else if(c != '\r') {
      uint8_t first = pgm_read_byte(&gfxFont->first);
      if((c >= first) && (c <= (uint8_t)pgm_read_byte(&gfxFont->last))) {
        uint8_t   c2    = c - pgm_read_byte(&gfxFont->first);
        GFXglyph *glyph = &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c2]);
        uint8_t   w     = pgm_read_byte(&glyph->width),
                  h     = pgm_read_byte(&glyph->height);
        if((w > 0) && (h > 0)) { // Is there an associated bitmap?
          int16_t xo = (int8_t)pgm_read_byte(&glyph->xOffset); // sic
          if(wrap && ((cursor_x + textsize * (xo + w)) >= _width)) {
            // Drawing character would go off right edge; wrap to new line
            cursor_x  = 0;
            cursor_y += (int16_t)textsize *
                        (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
          }
          drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
        }
        cursor_x += pgm_read_byte(&glyph->xAdvance) * (int16_t)textsize;
      }
    }

  }
  return 1;
}

// Draw a character
void Ayarafun_ST7789H2::drawChar(int16_t x, int16_t y, unsigned char c,
 uint16_t color, uint16_t bg, uint8_t size) {

  if(!gfxFont) { // 'Classic' built-in font

    if((x >= _width)            || // Clip right
       (y >= _height)           || // Clip bottom
       ((x + 6 * size - 1) < 0) || // Clip left
       ((y + 8 * size - 1) < 0))   // Clip top
      return;

    if(!_cp437 && (c >= 176)) c++; // Handle 'classic' charset behavior

    for(int8_t i=0; i<6; i++ ) {
      uint8_t line;
      if(i < 5) line = pgm_read_byte(font+(c*5)+i);
      else      line = 0x0;
      for(int8_t j=0; j<8; j++, line >>= 1) {
        if(line & 0x1) {
          if(size == 1) drawPixel(x+i, y+j, color);
          else          fillRect(x+(i*size), y+(j*size), size, size, color);
        } else if(bg != color) {
          if(size == 1) drawPixel(x+i, y+j, bg);
          else          fillRect(x+i*size, y+j*size, size, size, bg);
        }
      }
    }

  } else { // Custom font

    // Character is assumed previously filtered by write() to eliminate
    // newlines, returns, non-printable characters, etc.  Calling drawChar()
    // directly with 'bad' characters of font may cause mayhem!

    c -= pgm_read_byte(&gfxFont->first);
    GFXglyph *glyph  = &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c]);
    uint8_t  *bitmap = (uint8_t *)pgm_read_pointer(&gfxFont->bitmap);

    uint16_t bo = pgm_read_word(&glyph->bitmapOffset);
    uint8_t  w  = pgm_read_byte(&glyph->width),
             h  = pgm_read_byte(&glyph->height),
             xa = pgm_read_byte(&glyph->xAdvance);
    int8_t   xo = pgm_read_byte(&glyph->xOffset),
             yo = pgm_read_byte(&glyph->yOffset);
    uint8_t  xx, yy, bits, bit = 0;
    int16_t  xo16, yo16;

    if(size > 1) {
      xo16 = xo;
      yo16 = yo;
    }

    // Todo: Add character clipping here

    // NOTE: THERE IS NO 'BACKGROUND' COLOR OPTION ON CUSTOM FONTS.
    // THIS IS ON PURPOSE AND BY DESIGN.  The background color feature
    // has typically been used with the 'classic' font to overwrite old
    // screen contents with new data.  This ONLY works because the
    // characters are a uniform size; it's not a sensible thing to do with
    // proportionally-spaced fonts with glyphs of varying sizes (and that
    // may overlap).  To replace previously-drawn text when using a custom
    // font, use the getTextBounds() function to determine the smallest
    // rectangle encompassing a string, erase the area with fillRect(),
    // then draw new text.  This WILL infortunately 'blink' the text, but
    // is unavoidable.  Drawing 'background' pixels will NOT fix this,
    // only creates a new set of problems.  Have an idea to work around
    // this (a canvas object type for MCUs that can afford the RAM and
    // displays supporting setAddrWindow() and pushColors()), but haven't
    // implemented this yet.

    for(yy=0; yy<h; yy++) {
      for(xx=0; xx<w; xx++) {
        if(!(bit++ & 7)) {
          bits = pgm_read_byte(&bitmap[bo++]);
        }
        if(bits & 0x80) {
          if(size == 1) {
            drawPixel(x+xo+xx, y+yo+yy, color);
          } else {
            fillRect(x+(xo16+xx)*size, y+(yo16+yy)*size, size, size, color);
          }
        }
        bits <<= 1;
      }
    }

  } // End classic vs custom font
}

void Ayarafun_ST7789H2::setCursor(int16_t x, int16_t y) {
  cursor_x = x;
  cursor_y = y;
}

int16_t Ayarafun_ST7789H2::getCursorX(void) const {
  return cursor_x;
}

int16_t Ayarafun_ST7789H2::getCursorY(void) const {
  return cursor_y;
}

void Ayarafun_ST7789H2::setTextSize(uint8_t s) {
  textsize = (s > 0) ? s : 1;
}

void Ayarafun_ST7789H2::setTextColor(uint16_t c) {
  // For 'transparent' background, we'll set the bg
  // to the same as fg instead of using a flag
  textcolor = textbgcolor = c;
}

void Ayarafun_ST7789H2::setTextColor(uint16_t c, uint16_t b) {
  textcolor   = c;
  textbgcolor = b;
}

void Ayarafun_ST7789H2::setTextWrap(boolean w) {
  wrap = w;
}

uint8_t Ayarafun_ST7789H2::getRotation(void) const {
  return rotation;
}


// Enable (or disable) Code Page 437-compatible charset.
// There was an error in glcdfont.c for the longest time -- one character
// (#176, the 'light shade' block) was missing -- this threw off the index
// of every character that followed it.  But a TON of code has been written
// with the erroneous character indices.  By default, the library uses the
// original 'wrong' behavior and old sketches will still work.  Pass 'true'
// to this function to use correct CP437 character values in your code.
void Ayarafun_ST7789H2::cp437(boolean x) {
  _cp437 = x;
}

void Ayarafun_ST7789H2::setFont(const GFXfont *f) {
  if(f) {          // Font struct pointer passed in?
    if(!gfxFont) { // And no current font struct?
      // Switching from classic to new font behavior.
      // Move cursor pos down 6 pixels so it's on baseline.
      cursor_y += 6;
    }
  } else if(gfxFont) { // NULL passed.  Current font struct defined?
    // Switching from new to classic font behavior.
    // Move cursor pos up 6 pixels so it's at top-left of char.
    cursor_y -= 6;
  }
  gfxFont = (GFXfont *)f;
}

// Pass string and a cursor position, returns UL corner and W,H.
void Ayarafun_ST7789H2::getTextBounds(char *str, int16_t x, int16_t y,
 int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h) {
  uint8_t c; // Current character

  *x1 = x;
  *y1 = y;
  *w  = *h = 0;

  if(gfxFont) {

    GFXglyph *glyph;
    uint8_t   first = pgm_read_byte(&gfxFont->first),
              last  = pgm_read_byte(&gfxFont->last),
              gw, gh, xa;
    int8_t    xo, yo;
    int16_t   minx = _width, miny = _height, maxx = -1, maxy = -1,
              gx1, gy1, gx2, gy2, ts = (int16_t)textsize,
              ya = ts * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);

    while((c = *str++)) {
      if(c != '\n') { // Not a newline
        if(c != '\r') { // Not a carriage return, is normal char
          if((c >= first) && (c <= last)) { // Char present in current font
            c    -= first;
            glyph = &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c]);
            gw    = pgm_read_byte(&glyph->width);
            gh    = pgm_read_byte(&glyph->height);
            xa    = pgm_read_byte(&glyph->xAdvance);
            xo    = pgm_read_byte(&glyph->xOffset);
            yo    = pgm_read_byte(&glyph->yOffset);
            if(wrap && ((x + (((int16_t)xo + gw) * ts)) >= _width)) {
              // Line wrap
              x  = 0;  // Reset x to 0
              y += ya; // Advance y by 1 line
            }
            gx1 = x   + xo * ts;
            gy1 = y   + yo * ts;
            gx2 = gx1 + gw * ts - 1;
            gy2 = gy1 + gh * ts - 1;
            if(gx1 < minx) minx = gx1;
            if(gy1 < miny) miny = gy1;
            if(gx2 > maxx) maxx = gx2;
            if(gy2 > maxy) maxy = gy2;
            x += xa * ts;
          }
        } // Carriage return = do nothing
      } else { // Newline
        x  = 0;  // Reset x
        y += ya; // Advance y by 1 line
      }
    }
    // End of string
    *x1 = minx;
    *y1 = miny;
    if(maxx >= minx) *w  = maxx - minx + 1;
    if(maxy >= miny) *h  = maxy - miny + 1;

  } else { // Default font

    uint16_t lineWidth = 0, maxWidth = 0; // Width of current, all lines

    while((c = *str++)) {
      if(c != '\n') { // Not a newline
        if(c != '\r') { // Not a carriage return, is normal char
          if(wrap && ((x + textsize * 6) >= _width)) {
            x  = 0;            // Reset x to 0
            y += textsize * 8; // Advance y by 1 line
            if(lineWidth > maxWidth) maxWidth = lineWidth; // Save widest line
            lineWidth  = textsize * 6; // First char on new line
          } else { // No line wrap, just keep incrementing X
            lineWidth += textsize * 6; // Includes interchar x gap
          }
        } // Carriage return = do nothing
      } else { // Newline
        x  = 0;            // Reset x to 0
        y += textsize * 8; // Advance y by 1 line
        if(lineWidth > maxWidth) maxWidth = lineWidth; // Save widest line
        lineWidth = 0;     // Reset lineWidth for new line
      }
    }
    // End of string
    if(lineWidth) y += textsize * 8; // Add height of last (or only) line
    *w = maxWidth - 1;               // Don't include last interchar x gap
    *h = y - *y1;

  } // End classic vs custom font
}

// Same as above, but for PROGMEM strings
void Ayarafun_ST7789H2::getTextBounds(const __FlashStringHelper *str,
 int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h) {
  uint8_t *s = (uint8_t *)str, c;

  *x1 = x;
  *y1 = y;
  *w  = *h = 0;

  if(gfxFont) {

    GFXglyph *glyph;
    uint8_t   first = pgm_read_byte(&gfxFont->first),
              last  = pgm_read_byte(&gfxFont->last),
              gw, gh, xa;
    int8_t    xo, yo;
    int16_t   minx = _width, miny = _height, maxx = -1, maxy = -1,
              gx1, gy1, gx2, gy2, ts = (int16_t)textsize,
              ya = ts * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);

    while((c = pgm_read_byte(s++))) {
      if(c != '\n') { // Not a newline
        if(c != '\r') { // Not a carriage return, is normal char
          if((c >= first) && (c <= last)) { // Char present in current font
            c    -= first;
            glyph = &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c]);
            gw    = pgm_read_byte(&glyph->width);
            gh    = pgm_read_byte(&glyph->height);
            xa    = pgm_read_byte(&glyph->xAdvance);
            xo    = pgm_read_byte(&glyph->xOffset);
            yo    = pgm_read_byte(&glyph->yOffset);
            if(wrap && ((x + (((int16_t)xo + gw) * ts)) >= _width)) {
              // Line wrap
              x  = 0;  // Reset x to 0
              y += ya; // Advance y by 1 line
            }
            gx1 = x   + xo * ts;
            gy1 = y   + yo * ts;
            gx2 = gx1 + gw * ts - 1;
            gy2 = gy1 + gh * ts - 1;
            if(gx1 < minx) minx = gx1;
            if(gy1 < miny) miny = gy1;
            if(gx2 > maxx) maxx = gx2;
            if(gy2 > maxy) maxy = gy2;
            x += xa * ts;
          }
        } // Carriage return = do nothing
      } else { // Newline
        x  = 0;  // Reset x
        y += ya; // Advance y by 1 line
      }
    }
    // End of string
    *x1 = minx;
    *y1 = miny;
    if(maxx >= minx) *w  = maxx - minx + 1;
    if(maxy >= miny) *h  = maxy - miny + 1;

  } else { // Default font

    uint16_t lineWidth = 0, maxWidth = 0; // Width of current, all lines

    while((c = pgm_read_byte(s++))) {
      if(c != '\n') { // Not a newline
        if(c != '\r') { // Not a carriage return, is normal char
          if(wrap && ((x + textsize * 6) >= _width)) {
            x  = 0;            // Reset x to 0
            y += textsize * 8; // Advance y by 1 line
            if(lineWidth > maxWidth) maxWidth = lineWidth; // Save widest line
            lineWidth  = textsize * 6; // First char on new line
          } else { // No line wrap, just keep incrementing X
            lineWidth += textsize * 6; // Includes interchar x gap
          }
        } // Carriage return = do nothing
      } else { // Newline
        x  = 0;            // Reset x to 0
        y += textsize * 8; // Advance y by 1 line
        if(lineWidth > maxWidth) maxWidth = lineWidth; // Save widest line
        lineWidth = 0;     // Reset lineWidth for new line
      }
    }
    // End of string
    if(lineWidth) y += textsize * 8; // Add height of last (or only) line
    *w = maxWidth - 1;               // Don't include last interchar x gap
    *h = y - *y1;

  } // End classic vs custom font
}

// Return the size of the display (per current rotation)
int16_t Ayarafun_ST7789H2::width(void) const {
  return _width;
}

int16_t Ayarafun_ST7789H2::height(void) const {
  return _height;
}

////////// stuff not actively being used, but kept for posterity
/*

 uint8_t Ayarafun_ST7789H2::spiread(void) {
 uint8_t r = 0;
 if (_sid > 0) {
 r = shiftIn(_sid, _sclk, MSBFIRST);
 } else {
 //SID_DDR &= ~_BV(SID);
 //int8_t i;
 //for (i=7; i>=0; i--) {
 //  SCLK_PORT &= ~_BV(SCLK);
 //  r <<= 1;
 //  r |= (SID_PIN >> SID) & 0x1;
 //  SCLK_PORT |= _BV(SCLK);
 //}
 //SID_DDR |= _BV(SID);
 
 }
 return r;
 }
 
 
 void Ayarafun_ST7789H2::dummyclock(void) {
 
 if (_sid > 0) {
 digitalWrite(_sclk, LOW);
 digitalWrite(_sclk, HIGH);
 } else {
 // SCLK_PORT &= ~_BV(SCLK);
 //SCLK_PORT |= _BV(SCLK);
 }
 }
 uint8_t Ayarafun_ST7789H2::readdata(void) {
 *portOutputRegister(rsport) |= rspin;
 
 *portOutputRegister(csport) &= ~ cspin;
 
 uint8_t r = spiread();
 
 *portOutputRegister(csport) |= cspin;
 
 return r;
 
 } 
 
 uint8_t Ayarafun_ST7789H2::readcommand8(uint8_t c) {
 digitalWrite(_rs, LOW);
 
 *portOutputRegister(csport) &= ~ cspin;
 
 spiwrite(c);
 
 digitalWrite(_rs, HIGH);
 pinMode(_sid, INPUT); // input!
 digitalWrite(_sid, LOW); // low
 spiread();
 uint8_t r = spiread();
 
 
 *portOutputRegister(csport) |= cspin;
 
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 
 uint16_t Ayarafun_ST7789H2::readcommand16(uint8_t c) {
 digitalWrite(_rs, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 uint16_t r = spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 uint32_t Ayarafun_ST7789H2::readcommand32(uint8_t c) {
 digitalWrite(_rs, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 
 dummyclock();
 dummyclock();
 
 uint32_t r = spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 */
