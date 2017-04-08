/* ------------------------------------------------------------------------- *
   @file              : Ayarafun_ST7789H2.h
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
 #ifndef _AYARAFUN_ST7789H2H_
#define _AYARAFUN_ST7789H2H_
#include "Arduino.h"
#include "Print.h"

#include "gfxfont.h"
#include "glcdfont.c"

#ifdef __AVR
  #include <avr/pgmspace.h>
//#elif defined(__arm__)
//  #include <pgmspace.h>
#elif defined(__esp8266__)
  #include <pgmspace.h>
#elif defined(ESP32)
  #include <pgmspace.h>
#endif

#define ST7789H2_TFTWIDTH   240
#define ST7789H2_TFTHEIGHT  240 //204

// SYSTEM FUNCTION COMMAND TABLE 1
#define ST7789H2_NOP        0x00
#define ST7789H2_SWRESET    0x01    // Software Reset
//#define ST7789H2_RDDID      0x04    // Read Display ID
//#define ST7789H2_RDDST      0x09    // Read Display Status
//#define ST7789H2_RDDPM      0x0A    // Read Display Power Mode
//#define ST7789H2_RDDMADCTL  0x0B    // Read Display MADCTL
//#define ST7789H2_RDDCOLMOD  0x0C    // Read Display Pixel Format
//#define ST7789H2_RDDIM      0x0D    // Read Display Image Mode
//#define ST7789H2_RDDSM      0x0E    // Read Display Signal Mode
//#define ST7789H2_RDDSDR     0x0F    // Read Display Self-Diagnostic Result
#define ST7789H2_SLPIN      0x10    // Sleep in
#define ST7789H2_SLPOUT     0x11    // Sleep Out *
//#define ST7789H2_PTLON      0x12    // Partial Display Mode On
//#define ST7789H2_NORON      0x13    // Normal Display Mode On
#define ST7789H2_INVOFF     0x20    // Display Inversion Off *
#define ST7789H2_INVON      0x21    // Display Inversion On *
//#define ST7789H2_GAMSET     0x26    // Gamma Set
#define ST7789H2_DISPOFF    0x28    // Display Off
#define ST7789H2_DISPON     0x29    // Display On *
#define ST7789H2_CASET      0x2A    // Column Address Set *
#define ST7789H2_RASET      0x2B    // Row Address Set *
#define ST7789H2_RAMWR      0x2C    // Memory Write *
//#define ST7789H2_RAMRD      0x2E    // Memory Read
//#define ST7789H2_PTLAR      0x30    // Partial Area
//#define ST7789H2_VSCRDEF    0x33    // Vertical Scrolling Definition
//#define ST7789H2_TEOFF      0x34    // Tearing Effect Line OFF
//#define ST7789H2_TEON       0x35    // Tearing Effect Line On
#define ST7789H2_MADCTL     0x36    // Memory Data Access Control *
//#define ST7789H2_VSCSAD     0x37    // Vertical Scroll Start Address of RAM
//#define ST7789H2_IDMOFF     0x38    // Idle Mode Off
//#define ST7789H2_IDMON      0x39    // Idle mode on
#define ST7789H2_COLMOD     0x3A    // Interface Pixel Format *
//#define ST7789H2_WRMEMC     0x3C    // Write Memory Continue
//#define ST7789H2_RDMEMC     0x3E    // Read Memory Continue
//#define ST7789H2_STE        0x44    // Set Tear Scanline
//#define ST7789H2_GSCAN      0x45    // Get Scanline
//#define ST7789H2_WRDISBV    0x51    // Write Display Brightness
//#define ST7789H2_RDDISBV    0x52    // Read Display Brightness Value
//#define ST7789H2_WRCTRLD    0x53    // Write CTRL Display
//#define ST7789H2_RDCTRLD    0x54    // Read CTRL Value Display
//#define ST7789H2_WRCACE     0x55    // Write Content Adaptive Brightness Control and Color Enhancement
//#define ST7789H2_RDCABC     0x56    // Read Content Adaptive Brightness Control
//#define ST7789H2_WRCABCMB   0x5E    // Write CABC Minimum Brightness
//#define ST7789H2_RDCABCMB   0x5F    // Read CABC Minimum Brightness
//#define ST7789H2_RDABCSDR   0x68    // Read Automatic Brightness Control Self-Diagnostic Result
//#define ST7789H2_RDID1      0xDA    // Read ID1
//#define ST7789H2_RDID2      0xDB    // Read ID2
//#define ST7789H2_RDID3      0xDC    // Read ID3

// SYSTEM FUNCTION COMMAND TABLE 2
//#define ST7789H2_RAMCTRL    0xB0    // RAM Control
#define ST7789H2_RGBCTRL    0xB1    // RGB Interface Control *
#define ST7789H2_PORCTRL    0xB2    // Porch Setting *
//#define ST7789H2_FRCTRL1    0xB3    // Frame Rate Control 1 (In partial mode/ idle colors)
//#define ST7789H2_PARCTRL    0xB5    // Partial Control
#define ST7789H2_GCTRL      0xB7    // Gate Control *
//#define ST7789H2_GTADJ      0xB8    // Gate On Timing Adjustment
//#define ST7789H2_DGMEN      0xBA    // Digital Gamma Enable
#define ST7789H2_VCOMS      0xBB    // VCOM Setting *
//#define ST7789H2_POWSAVE    0xBC    // Power Saving Mode
//#define ST7789H2_DLPOFFSAVE 0xBD    // Display off power save
#define ST7789H2_LCMCTRL    0xC0    // LCM Control *
//#define ST7789H2_IDSET      0xC1    // ID Code Setting
#define ST7789H2_VDVVRHEN   0xC2    // VDV and VRH Command Enable *
#define ST7789H2_VRHS       0xC3    // VRH Set *
#define ST7789H2_VDVS       0xC4    // VDV Set *
//#define ST7789H2_VCMOFSET   0xC5    // VCOM Offset Set
#define ST7789H2_FRCTRL2    0xC6    // Frame Rate Control in Normal Mode *
//#define ST7789H2_CABCCTRL   0xC7    // CABC Control
//#define ST7789H2_REGSEL1    0xC8    // Register Value Selection 1
//#define ST7789H2_REGSEL2    0xCA    // Register Value Selection 2
//#define ST7789H2_PWMFRSEL   0xCC    // PWM Frequency Selection
#define ST7789H2_PWCTRL1    0xD0    // Power Control 1 *
//#define ST7789H2_VAPVANEN   0xD2    // Enable VAP/VAN signal output
//#define ST7789H2_CMD2EN     0xDF    // Command 2 Enable
#define ST7789H2_PVGAMCTRL  0xE0    // Positive Voltage Gamma Control *
#define ST7789H2_NVGAMCTRL  0xE1    // Negative Voltage Gamma Control *
//#define ST7789H2_DGMLUTR    0xE2    // Digital Gamma Look-up Table for Red
//#define ST7789H2_DGMLUTB    0xE3    // Digital Gamma Look-up Table for Blue
//#define ST7789H2_GATECTRL   0xE4    // Gate Control
//#define ST7789H2_SPI2EN     0xE7    // SPI2 Enable
//#define ST7789H2_PWCTRL2    0xE8    // Power Control 2
//#define ST7789H2_EQCTRL     0xE9    // Equalize time control
//#define ST7789H2_PROMCTRL   0xEC    // Program Mode Control
//#define ST7789H2_PROMEN     0xFA    // Program Mode Enable
//#define ST7789H2_NVMSET     0xFC    // NVM Setting
//#define ST7789H2_PROMACT    0xFE    // Program action

// VGA Color Palette
#define VGA_BLACK       0x0000      /*   0,   0,   0 */
#define VGA_NAVY        0x000F      /*   0,   0, 128 */
#define VGA_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define VGA_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define VGA_MAROON      0x7800      /* 128,   0,   0 */
#define VGA_PURPLE      0x780F      /* 128,   0, 128 */
#define VGA_OLIVE       0x7BE0      /* 128, 128,   0 */
#define VGA_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define VGA_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define VGA_BLUE        0x001F      /*   0,   0, 255 */
#define VGA_GREEN       0x07E0      /*   0, 255,   0 */
#define VGA_CYAN        0x07FF      /*   0, 255, 255 */
#define VGA_RED         0xF800      /* 255,   0,   0 */
#define VGA_MAGENTA     0xF81F      /* 255,   0, 255 */
#define VGA_YELLOW      0xFFE0      /* 255, 255,   0 */
#define VGA_WHITE       0xFFFF      /* 255, 255, 255 */
#define VGA_ORANGE      0xFD20      /* 255, 165,   0 */
#define VGA_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define VGA_PINK        0xF81F

class Ayarafun_ST7789H2 : public Print {

 public:

  Ayarafun_ST7789H2(int8_t CS, int8_t RS, int8_t SID, int8_t SCLK, int8_t BL, int8_t RST = -1);
  Ayarafun_ST7789H2(int8_t CS, int8_t RS, int8_t BL, int8_t RST = -1);

  void     begin(void),
           initial(void),
		   reset(void),
           setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1),
           pushColor(uint16_t color),
           fillScreen(uint16_t color),
           drawPixel(int16_t x, int16_t y, uint16_t color),
           drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color),
           drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color),
           fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color),
           setRotation(uint8_t r),
		   backlight(boolean i),
           invertDisplay(boolean i);
  uint16_t Color565(uint8_t r, uint8_t g, uint8_t b);

  /* These are not for current use, 8-bit protocol only!
  uint8_t  readdata(void),
           readcommand8(uint8_t);
  uint16_t readcommand16(uint8_t);
  uint32_t readcommand32(uint8_t);
  void     dummyclock(void);
  */

  // These MAY be overridden by the subclass to provide device-specific
  // optimized code.  Otherwise 'generic' versions are used.
  virtual void
    drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color),
//    drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color),
//    drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color),
    drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
//    fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color),
//    fillScreen(uint16_t color);
//    invertDisplay(boolean i);
    void
    drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color),
    drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,
      uint16_t color),
    fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color),
    fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,
      int16_t delta, uint16_t color),
    drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
      int16_t x2, int16_t y2, uint16_t color),
    fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
      int16_t x2, int16_t y2, uint16_t color),
    drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h,
      int16_t radius, uint16_t color),
    fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h,
      int16_t radius, uint16_t color),
    drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap,
      int16_t w, int16_t h, uint16_t color),
    drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap,
      int16_t w, int16_t h, uint16_t color, uint16_t bg),
    drawBitmap(int16_t x, int16_t y, uint8_t *bitmap,
      int16_t w, int16_t h, uint16_t color),
    drawBitmap(int16_t x, int16_t y, uint8_t *bitmap,
      int16_t w, int16_t h, uint16_t color, uint16_t bg),
    drawXBitmap(int16_t x, int16_t y, const uint8_t *bitmap,
      int16_t w, int16_t h, uint16_t color),
    drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color,
      uint16_t bg, uint8_t size),
    setCursor(int16_t x, int16_t y),
    setTextColor(uint16_t c),
    setTextColor(uint16_t c, uint16_t bg),
    setTextSize(uint8_t s),
    setTextWrap(boolean w),
//    setRotation(uint8_t r),
    cp437(boolean x=true),
    setFont(const GFXfont *f = NULL),
    getTextBounds(char *string, int16_t x, int16_t y,
      int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h),
    getTextBounds(const __FlashStringHelper *s, int16_t x, int16_t y,
      int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h);

  virtual size_t write(uint8_t);

  int16_t height(void) const;
  int16_t width(void) const;

  uint8_t getRotation(void) const;

  // get current cursor position (get rotation safe maximum values, using: width() for x, height() for y)
  int16_t getCursorX(void) const;
  int16_t getCursorY(void) const;

  
 private:
  uint8_t  tabcolor;

  void     spiwrite(uint8_t),
           spiwrite16(uint16_t v),
           writecommand(uint8_t c),
           writedata(uint8_t d),
           writedata16(uint16_t d),
           writeColor(uint16_t color, uint16_t count),
           setCS(bool level),
           setRS(bool level),
           commandList(const uint8_t *addr),
           commonInit(const uint8_t *cmdList);
//uint8_t  spiread(void);

  boolean  hwSPI;

#if defined(__AVR__) || defined(CORE_TEENSY)
volatile uint8_t *dataport, *clkport, *csport, *rsport;
  uint8_t  _cs, _rs, _rst, _sid, _sclk, _bl,
           datapinmask, clkpinmask, cspinmask, rspinmask;
#elif defined(__SAM3X8E__)
  Pio *dataport, *clkport, *csport, *rsport;
  uint32_t  _cs, _rs, _rst, _sid, _sclk, _bl,
            datapinmask, clkpinmask, cspinmask, rspinmask;
#elif defined(ESP32)
  volatile uint32_t *sidport, *clkport, *rsport, *csport;
  int32_t  _cs, _rs, _rst, _sid, _sclk, _bl;
  uint32_t  sidpinmask, clkpinmask, cspinmask, rspinmask;
#elif defined(ARDUINO_ARCH_ESP8266)
#define ST7789H2_USE_GENERIC_IO
#define ST7789H2_USE_HWSPI_ONLY
#define ST7789H2_USE_HWSPI_WRITE16
#define ST7789H2_USE_HWSPI_WRITEPATTERN
  uint8_t _cs, _rs, _rst, _sid, _sclk, _bl;
#endif

 protected:
  //const int16_t
  //  WIDTH, HEIGHT;   // This is the 'raw' display w/h - never changes
  int16_t
    _width, _height, // Display w/h as modified by current rotation
    cursor_x, cursor_y;
  uint16_t
    textcolor, textbgcolor;
  uint8_t
    textsize,
    rotation;
  boolean
    wrap,   // If set, 'wrap' text at right edge of display
    _cp437; // If set, use correct CP437 charset (default is off)
  GFXfont
    *gfxFont;
};

#endif