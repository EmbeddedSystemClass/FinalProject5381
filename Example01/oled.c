/*
 * oled.c
 *
 *  Created on: Sep 8, 2016
 *      Author: Mike Mehr
 *
 *  This file will contain the high-level code to interface to the adapted SSD-1306 Graphics lib.
 */
#include "oled.h"
//#include "spi.h"
#include "stdio.h" // for printf() debugging
#include "string.h"
#include "SSD1306.h"

#include "logo.h"
//#include "gfxfont.h"

static uint8_t gfxcolor = WHITE;
static boolean immediate = false;

// set up the pins on the LPC-1769 via the class into the SPI implementation (doesn't work for I2C for now)
// will return 0 if cannot instantiate the class, 1 if it's good to go
int oled_init(void) {
	gfxcolor  = WHITE;
	immediate = false;
	gfx_setup_defaults(SSD1306_LCDWIDTH, SSD1306_LCDHEIGHT);
	ssd1306_init_hwspi(OLED_DC, OLED_RST, OLED_CS);
	return 0;
}

int oled_begin(int reset) {
	// perform the reset if asked, return result
	// ALSO NOTE: this sets up the pin directions on DC and CS, and RST if a reset is requested
	// it assumes SPI has already been set up by someone else (i.e., the BMPE driver)
	// it then sets up SPI transfer mode and sends a series of commands to the device to initialize it
	ssd1306_begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS, (reset==1), true);
	ssd1306_clearDisplay();
	gfx_drawBitmap(0, 0, logoBitmap, logoWidthPages * 8, logoHeightPixels, 1, 0);
	ssd1306_display(); // send RAM buffer to the hardware (initially set up with logo)
	return 1;
}

//enum { DRAWMODE_DEFERRED, DRAWMODE_IMMEDIATE };
void  oled_setDrawMode(int new_setting) { immediate = (new_setting == 0 ? false : true); }
int  oled_getDrawMode(void) { return immediate? 1: 0; }
//enum { TEXTWRAP_OFF, TEXTWRAP_ON };
void  oled_setTextWrap(int new_setting) { gfx_setTextWrap(new_setting == 0 ? false : true); }
int  oled_getTextWrap(void) { return gfx_getTextWrap()? 1: 0; }
//enum { BLACK, WHITE, INVERT };
void  oled_setGraphicsColor(int new_setting) { gfxcolor = (new_setting); }
int  oled_getGraphicsColor(void) { return gfxcolor; }
//enum { NORMAL, INVERTED, NORMAL_OVL, INVERTED_OVL };
void  oled_setTextColor(int new_setting) {
	switch (new_setting) {
	case NORMAL:
		gfx_setTextColors(WHITE, BLACK);
		break;
	case INVERTED:
		gfx_setTextColors(BLACK, WHITE);
		break;
	case NORMAL_OVL:
		gfx_setTextColor(WHITE); // on transparent
		break;
	case INVERTED_OVL:
		gfx_setTextColor(BLACK);
		break;
	}
}
int  oled_getTextColor(void) {
	int textcolor = gfx_getTextColor();
	int textbgcolor = gfx_getTextBackgroundColor();
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
void  oled_setCursor(int x, int y) { gfx_setCursor(x, y); }
int  oled_getCursorX(void) { return gfx_getCursorX(); }
int  oled_getCursorY(void) { return gfx_getCursorY(); }
int  oled_getScreenW(void) { return gfx_getWidth(); }
int  oled_getScreenH(void) { return gfx_getHeight(); }

void oled_print(const char* str)
{
	gfx_print(str);
	if (immediate)
		ssd1306_display();
}

void oled_println(const char* str)
{
	gfx_println(str);
	if (immediate)
		ssd1306_display();
}

void oled_display(void) {
	ssd1306_display(); // to the hardware
}

void oled_clearDisplay(void) {
	ssd1306_clearDisplay(); // RAM buffer only

	if (immediate)
		ssd1306_display(); // to the hardware
	return;
}

void oled_invertDisplay(int inverted) {
	ssd1306_invertDisplay(inverted);
	return;
}

void oled_drawBitmap(const uint8_t* bitmap, unsigned short w, unsigned short h) {
	gfx_drawBitmap(gfx_getCursorX(), gfx_getCursorY(),
			bitmap, w, h,
			gfx_getTextColor(), gfx_getTextBackgroundColor());
	if (immediate)
		ssd1306_display();
}

