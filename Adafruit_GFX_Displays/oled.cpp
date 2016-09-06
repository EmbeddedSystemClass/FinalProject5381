/*
 * oled.cpp
 *
 *  Created on: Sep 1, 2016
 *      Author: Mike Mehr
 */

#define ARDUINO 10000 // MLM - joking around but needed

extern "C" {
#include "oled.h"
#include "spi.h"
#include "stdio.h" // for printf() debugging
}

#include "Adafruit_SSD1306.h"

/* The following "LOGO" was generated from a mono BMP file by The Dot Factory: http://www.eran.io/the-dot-factory-an-lcd-font-and-image-generator/
 * Postprocessing needed: rename "uint_8" to "uint8_t", add square brackets after bitmap name - this can be saved as a preset in the tool
 * It can also be used to generate fonts for use in a certain kind of driver - however there are many fonts already created here using
 * a different format that we don't have to write drivers for. So for now we can use it for bitmap conversion.
*/
//  Image data for himom
//

//const uint8_t himomBitmap[] =
//{
//	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, //                                                                                                         ###
//	0xE0, 0x38, 0x70, 0x00, 0x07, 0xC0, 0x0F, 0x80, 0x7F, 0x01, 0xF0, 0x03, 0xE0, 0xE0, // ###       ###    ###                 #####          #####        #######       #####          #####     ###
//	0xE0, 0x38, 0x70, 0x00, 0x07, 0xE0, 0x1F, 0x81, 0xFF, 0x81, 0xF8, 0x07, 0xE0, 0xE0, // ###       ###    ###                 ######        ######      ##########      ######        ######     ###
//	0xE0, 0x38, 0x70, 0x00, 0x07, 0xE0, 0x1F, 0x83, 0xFF, 0xC1, 0xF8, 0x07, 0xE0, 0xE0, // ###       ###    ###                 ######        ######     ############     ######        ######     ###
//	0xE0, 0x38, 0x70, 0x00, 0x07, 0xF0, 0x1F, 0x87, 0xC1, 0xE1, 0xFC, 0x07, 0xE0, 0xE0, // ###       ###    ###                 #######       ######    #####     ####    #######       ######     ###
//	0xE0, 0x38, 0x70, 0x00, 0x07, 0x70, 0x3B, 0x87, 0x00, 0xE1, 0xDC, 0x0E, 0xE0, 0xE0, // ###       ###    ###                 ### ###      ### ###    ###        ###    ### ###      ### ###     ###
//	0xE0, 0x38, 0x70, 0x00, 0x07, 0x70, 0x3B, 0x8F, 0x00, 0x71, 0xDC, 0x0E, 0xE0, 0xE0, // ###       ###    ###                 ### ###      ### ###   ####         ###   ### ###      ### ###     ###
//	0xE0, 0x38, 0x70, 0x00, 0x07, 0x78, 0x7B, 0x8E, 0x00, 0x71, 0xDE, 0x1E, 0xE0, 0xE0, // ###       ###    ###                 ### ####    #### ###   ###          ###   ### ####    #### ###     ###
//	0xFF, 0xF8, 0x70, 0x00, 0x07, 0x38, 0x73, 0x8E, 0x00, 0x71, 0xCE, 0x1C, 0xE0, 0xE0, // #############    ###                 ###  ###    ###  ###   ###          ###   ###  ###    ###  ###     ###
//	0xFF, 0xF8, 0x70, 0x00, 0x07, 0x3C, 0x73, 0x8E, 0x00, 0x71, 0xCF, 0x1C, 0xE0, 0xE0, // #############    ###                 ###  ####   ###  ###   ###          ###   ###  ####   ###  ###     ###
//	0xFF, 0xF8, 0x70, 0x00, 0x07, 0x3C, 0xE3, 0x8E, 0x00, 0x71, 0xCF, 0x38, 0xE0, 0xE0, // #############    ###                 ###  ####  ###   ###   ###          ###   ###  ####  ###   ###     ###
//	0xE0, 0x38, 0x70, 0x00, 0x07, 0x1C, 0xE3, 0x8E, 0x00, 0x71, 0xC7, 0x38, 0xE0, 0xE0, // ###       ###    ###                 ###   ###  ###   ###   ###          ###   ###   ###  ###   ###     ###
//	0xE0, 0x38, 0x70, 0x00, 0x07, 0x1F, 0xE3, 0x8E, 0x00, 0xF1, 0xC7, 0xF8, 0xE0, 0xE0, // ###       ###    ###                 ###   ########   ###   ###         ####   ###   ########   ###     ###
//	0xE0, 0x38, 0x70, 0x00, 0x07, 0x0F, 0xC3, 0x87, 0x00, 0xE1, 0xC3, 0xF0, 0xE0, 0x00, // ###       ###    ###                 ###    ######    ###    ###        ###    ###    ######    ###
//	0xE0, 0x38, 0x70, 0xE0, 0x07, 0x0F, 0xC3, 0x87, 0x83, 0xE1, 0xC3, 0xF0, 0xE0, 0xE0, // ###       ###    ###    ###          ###    ######    ###    ####     #####    ###    ######    ###     ###
//	0xE0, 0x38, 0x70, 0xE0, 0x07, 0x0F, 0x83, 0x83, 0xFF, 0xC1, 0xC3, 0xE0, 0xE0, 0xE0, // ###       ###    ###    ###          ###    #####     ###     ############     ###    #####     ###     ###
//	0xE0, 0x38, 0x70, 0xE0, 0x07, 0x07, 0x83, 0x81, 0xFF, 0x81, 0xC1, 0xE0, 0xE0, 0xE0, // ###       ###    ###    ###          ###     ####     ###      ##########      ###     ####     ###     ###
//	0xE0, 0x38, 0x70, 0xE0, 0x07, 0x07, 0x83, 0x80, 0xFE, 0x01, 0xC1, 0xE0, 0xE0, 0xE0, // ###       ###    ###    ###          ###     ####     ###       #######        ###     ####     ###     ###
//	0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                         ##
//	0x00, 0x00, 0x01, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                        ###
//	0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                       ###
//	0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                       ##
//};
//
//// Bitmap sizes for himom
//const uint8_t himomWidthPages = 14;
//const uint8_t himomHeightPixels = 22;
#include "logo.h"
extern const uint8_t font[]; // internal standard 5x7 font

// internal drawing module (replaces Adafruit libraries as C code)
static void setup_defaults(int16_t w, int16_t h);
static void drawBitmap(uint16_t x, uint16_t y, const uint8_t* bitmap, uint16_t w, uint16_t h, uint16_t color, uint16_t bgcolor);
// text printing module forward references
static void print(const char str[]);
static void println(const char str[]);
static void setCursor(int16_t x, int16_t y);
static int16_t getCursorX(void);
static int16_t getCursorY(void);
static void setTextSize(uint8_t s);
static void setTextColor(uint16_t c);
static void setTextColors(uint16_t c, uint16_t b);
static void setTextWrap(boolean w);
static uint8_t getRotation(void);
static void setRotation(uint8_t x);

Adafruit_SSD1306 OLED(OLED_DC, OLED_RST, OLED_CS); // TODO: will eventually be replaced

// set up the pins on the LPC-1769 via the class into the SPI implementation (doesn't work for I2C for now)
// will return 0 if cannot instantiate the class, 1 if it's good to go
int oled_init(void) {
	// set up the I/O pins on the LPC board via CMSIS
	// NOTE: assumes SPI0 is already set up for device sharing, so we only need to add these 3 pins to the mix
	spi_pinConfig(OLED_DC, 0); // use GPIO (func.0)
	spi_pinConfig(OLED_RST, 0);
	spi_pinConfig(OLED_CS, 0);
	setup_defaults(SSD1306_LCDWIDTH, SSD1306_LCDHEIGHT);
	OLED.init(OLED_DC, OLED_RST, OLED_CS);
	return 0;
}

int oled_begin(int reset) {
	// perform the reset if asked, return result
	// ALSO NOTE: this sets up the pin directions on DC and CS, and RST if a reset is requested
	// it assumes SPI has already been set up by someone else (i.e., the BMPE driver)
	// it then sets up SPI transfer mode and sends a series of commands to the device to initialize it
	OLED.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS, (reset==1), true);
	OLED.clearDisplay();
	drawBitmap(0, 0, logoBitmap, logoWidthPages * 8, logoHeightPixels, 1, 0);
	OLED.display(); // send RAM buffer to the hardware (initially set up with logo)
	return 1;
}

/// INCLUDE STRING PRINTING SUPPORT FROM GFX LIBRARY AND TRANSLATE TO C
static int16_t
    WIDTH, HEIGHT;   // This is the 'raw' display w/h - never changes
static   int16_t
    _width, _height, // Display w/h as modified by current rotation
    cursor_x, cursor_y;
static   uint16_t
    textcolor, textbgcolor;
static   uint8_t
    textsize,
	gfxcolor,
    rotation;
static   boolean
    wrap,   // If set, 'wrap' text at right edge of display
	immediate, // if set, send buffer to display after every graphics call
    _cp437; // If set, use correct CP437 charset (default is off)
static   GFXfont
    *gfxFont;

//enum { DRAWMODE_DEFERRED, DRAWMODE_IMMEDIATE };
void  oled_setDrawMode(int new_setting) { immediate = (new_setting == 0 ? false : true); }
int  oled_getDrawMode(void) { return immediate? 1: 0; }
//enum { TEXTWRAP_OFF, TEXTWRAP_ON };
void  oled_setTextWrap(int new_setting) { setTextWrap(new_setting == 0 ? false : true); }
int  oled_getTextWrap(void) { return wrap? 1: 0; }
//enum { BLACK, WHITE, INVERT };
void  oled_setGraphicsColor(int new_setting) { gfxcolor = (new_setting); }
int  oled_getGraphicsColor(void) { return gfxcolor; }
//enum { NORMAL, INVERTED, NORMAL_OVL, INVERTED_OVL };
void  oled_setTextColor(int new_setting) {
	switch (new_setting) {
	case NORMAL:
		setTextColors(WHITE, BLACK);
		break;
	case INVERTED:
		setTextColors(BLACK, WHITE);
		break;
	case NORMAL_OVL:
		setTextColor(WHITE); // on transparent
		break;
	case INVERTED_OVL:
		setTextColor(BLACK);
		break;
	}
}
int  oled_getTextColor(void) {
	if (textcolor == WHITE && textbgcolor == BLACK)
		return NORMAL;
	if (textbgcolor == WHITE && textcolor == BLACK)
		return INVERTED;
	if (textcolor == WHITE && textbgcolor == WHITE)
		return NORMAL_OVL;
	if (textbgcolor == BLACK && textcolor == BLACK)
		return INVERTED_OVL;
	return -1;
}
// graphics cursor functions
void  oled_setCursor(int x, int y) { setCursor(x, y); }
int  oled_getCursorX(void) { return getCursorX(); }
int  oled_getCursorY(void) { return getCursorY(); }
int  oled_getScreenW(void) { return _width; }
int  oled_getScreenH(void) { return _height; }

void oled_print(const char* str)
{
	print(str);
	if (immediate)
		OLED.display();
}

void oled_println(const char* str)
{
	println(str);
	if (immediate)
		OLED.display();
}

void oled_display(void) {
	OLED.display(); // to the hardware
}

void oled_clearDisplay(void) {
	OLED.clearDisplay(); // RAM buffer only
//	drawBitmap(10, 20, himomBitmap, himomWidthPages * 8, himomHeightPixels, 1, 0);
////	OLED.drawCircle(32, 32, 16, 1);
////	OLED.drawRect(0, 0, 10, 10, 1);
////	OLED.drawLine(0, 0, 10, 10, 1);
//	setTextSize(1);
//	setTextColor(WHITE);
//	setCursor(0,0);
//	println("Hello, Mom!"); // crashes due to C++ virtual function tables not being initialized!

	if (immediate)
		OLED.display(); // to the hardware
	return;
}

void oled_invertDisplay(int inverted) {
	OLED.invertDisplay(inverted);
	return;
}

void oled_drawBitmap(const uint8_t* bitmap, int w, int h) {
	drawBitmap(cursor_x, cursor_y,
			bitmap, w, h,
			textcolor, textbgcolor);
	if (immediate)
		OLED.display();
}

static void drawBitmap(uint16_t x, uint16_t y, const uint8_t* bitmap, uint16_t w, uint16_t h, uint16_t color, uint16_t bg) {
	  int16_t i, j, byteWidth = (w + 7) / 8;
	  uint8_t byte;

	  for(j=0; j<h; j++) {
	    for(i=0; i<w; i++) {
	      if(i & 7) byte <<= 1;
	      else      byte   = bitmap[j * byteWidth + i / 8];
	      if(byte & 0x80) OLED.drawPixel(x+i, y+j, color);
	      else if (bg != -1) OLED.drawPixel(x+i, y+j, bg);
	    }
	  }
}

static void setup_defaults(int16_t w, int16_t h)
{
	WIDTH = w;
	HEIGHT = h;
	_width    = WIDTH;
	_height   = HEIGHT;
	rotation  = 0;
	cursor_y  = cursor_x    = 0;
	textsize  = 1;
	textcolor = textbgcolor = 0xFFFF;
	wrap      = true;
	_cp437    = true; // no need for old behavior
	gfxFont   = NULL;
	gfxcolor  = WHITE;
	immediate = false;
}

static void setCursor(int16_t x, int16_t y) {
  cursor_x = x;
  cursor_y = y;
}

int16_t getCursorX(void) {
  return cursor_x;
}

int16_t getCursorY(void) {
  return cursor_y;
}

static void setTextSize(uint8_t s) {
  textsize = (s > 0) ? s : 1;
}

static void setTextColor(uint16_t c) {
  // For 'transparent' background, we'll set the bg
  // to the same as fg instead of using a flag
  textcolor = textbgcolor = c;
}

static void setTextColors(uint16_t c, uint16_t b) {
  textcolor   = c;
  textbgcolor = b;
}

static void setTextWrap(boolean w) {
  wrap = w;
}

static uint8_t getRotation(void) {
  return rotation;
}

static void setRotation(uint8_t x) {
  rotation = (x & 3);
  switch(rotation) {
   case 0:
   case 2:
    _width  = WIDTH;
    _height = HEIGHT;
    break;
   case 1:
   case 3:
    _width  = HEIGHT;
    _height = WIDTH;
    break;
  }
}

// Enable (or disable) Code Page 437-compatible charset.
// There was an error in glcdfont.c for the longest time -- one character
// (#176, the 'light shade' block) was missing -- this threw off the index
// of every character that followed it.  But a TON of code has been written
// with the erroneous character indices.  By default, the library uses the
// original 'wrong' behavior and old sketches will still work.  Pass 'true'
// to this function to use correct CP437 character values in your code.
static void cp437(boolean x) {
  _cp437 = x;
}

static void setFont(const GFXfont *f) {
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

static uint8_t pgm_read_byte(const void* ptr) { return *(const uint8_t*)ptr; }
static uint16_t pgm_read_word(const void* ptr) { return *(const uint16_t*)ptr; }
static void* pgm_read_pointer(void* ptr) { return *(void**)ptr; }

// Draw a character
static void drawChar(int16_t x, int16_t y, unsigned char c,
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
          if(size == 1) OLED.drawPixel(x+i, y+j, color);
          else          OLED.fillRect(x+(i*size), y+(j*size), size, size, color);
        } else if(bg != color) {
          if(size == 1) OLED.drawPixel(x+i, y+j, bg);
          else          OLED.fillRect(x+i*size, y+j*size, size, size, bg);
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
             h  = pgm_read_byte(&glyph->height); //,
             //xa = pgm_read_byte(&glyph->xAdvance);
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
        	  OLED.drawPixel(x+xo+xx, y+yo+yy, color);
          } else {
        	  OLED.fillRect(x+(xo16+xx)*size, y+(yo16+yy)*size, size, size, color);
          }
        }
        bits <<= 1;
      }
    }

  } // End classic vs custom font
}

static size_t write(uint8_t c) {

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

static size_t printBuf(const uint8_t *buffer, size_t size)
{
  size_t n = 0;
  while (size--) {
    if (write(*buffer++)) n++;
    else break;
  }
  return n;
}

static void print(const char str[]) {
	printBuf((const uint8_t *)str, strlen(str));
}

static void println(const char str[]) {
	print(str);
	print("\n");
}
