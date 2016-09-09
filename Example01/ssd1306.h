/*
 * ssd1306.h
 *
 *  Created on: Sep 8, 2016
 *      Author: Mike
 */

#ifndef SSD1306_H_
#define SSD1306_H_

#include "stdint.h"
#include "stdbool.h"
typedef bool boolean;

#define SSD1306_I2C_ADDRESS   0x3C  // 011110+SA0+RW - 0x3C or 0x3D
// Address for 128x32 is 0x3C
// Address for 128x64 is 0x3D (default) or 0x3C (if SA0 is grounded)

/*=========================================================================
    SSD1306 Displays
    -----------------------------------------------------------------------
    The driver is used in multiple displays (128x64, 128x32, etc.).
    Select the appropriate display below to create an appropriately
    sized framebuffer, etc.

    SSD1306_128_64  128x64 pixel display

    SSD1306_128_32  128x32 pixel display

    SSD1306_96_16

    -----------------------------------------------------------------------*/
   #define SSD1306_128_64
//   #define SSD1306_128_32
//   #define SSD1306_96_16
/*=========================================================================*/

#if defined SSD1306_128_64 && defined SSD1306_128_32
  #error "Only one SSD1306 display can be specified at once in SSD1306.h"
#endif
#if !defined SSD1306_128_64 && !defined SSD1306_128_32 && !defined SSD1306_96_16
  #error "At least one SSD1306 display must be specified in SSD1306.h"
#endif

#if defined SSD1306_128_64
  #define SSD1306_LCDWIDTH                  128
  #define SSD1306_LCDHEIGHT                 64
#endif
#if defined SSD1306_128_32
  #define SSD1306_LCDWIDTH                  128
  #define SSD1306_LCDHEIGHT                 32
#endif
#if defined SSD1306_96_16
  #define SSD1306_LCDWIDTH                  96
  #define SSD1306_LCDHEIGHT                 16
#endif

#define BLACK 0
#define WHITE 1
#define INVERSE 2

#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

// graphics forward references
extern void gfx_setup_defaults(int16_t w, int16_t h);
extern int16_t gfx_getHeight(void);
extern int16_t gfx_getWidth(void);
extern void gfx_setCursor(int16_t x, int16_t y);
extern int16_t gfx_getCursorX(void);
extern int16_t gfx_getCursorY(void);
extern void gfx_drawBitmap(uint16_t x, uint16_t y, const uint8_t* bitmap, uint16_t w, uint16_t h, uint16_t color, uint16_t bgcolor);
// text printing forward references
extern void gfx_print(const char str[]);
extern void gfx_println(const char str[]);
extern void gfx_setTextSize(uint8_t s);
extern uint8_t gfx_getTextSize(void);
extern void gfx_setTextColor(uint16_t c);
extern void gfx_setTextColors(uint16_t c, uint16_t b);
extern uint16_t gfx_getTextColor(void);
extern uint16_t gfx_getTextBackgroundColor(void);
extern void gfx_setTextWrap(bool w);
extern bool gfx_getTextWrap(void);
extern uint8_t gfx_getRotation(void);
extern void gfx_setRotation(uint8_t x);

// internal drawing module (replaces Adafruit libraries as C code)
extern void ssd1306_init_swspi(int8_t SID, int8_t SCLK, int8_t DC, int8_t RST, int8_t CS); // SW SPI
extern void ssd1306_init_hwspi(int8_t DC, int8_t RST, int8_t CS); // HS SPI
extern void ssd1306_init_i2c(int8_t RST); // I2C
extern void ssd1306_begin(uint8_t switchvcc, uint8_t i2caddr, bool reset, bool sharedSPI);
extern void ssd1306_command(uint8_t c);
extern void ssd1306_clearDisplay(void);
extern void ssd1306_invertDisplay(uint8_t i);
extern void ssd1306_display();
extern void ssd1306_startscrollright(uint8_t start, uint8_t stop);
extern void ssd1306_startscrollleft(uint8_t start, uint8_t stop);
extern void ssd1306_startscrolldiagright(uint8_t start, uint8_t stop);
extern void ssd1306_startscrolldiagleft(uint8_t start, uint8_t stop);
extern void ssd1306_stopscroll(void);
extern void ssd1306_dim(boolean dim);
extern void ssd1306_drawPixel(int16_t x, int16_t y, uint16_t color);
extern void ssd1306_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

// MLM DEBUG - get a single pixel bit (0/1) or -1 if params out of range
extern int ssd1306_getPixel(int16_t x, int16_t y);

#endif /* SSD1306_H_ */
