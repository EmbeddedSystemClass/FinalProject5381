/*
 * oled.h
 *
 *  Created on: Sep 1, 2016
 *      Author: Mike Mehr
 *      Purpose: Provides C interface to port of Adafruit SSD1306 and GFX library code to drive my LANMU 1.3" OLED display device.
 *
 *      The plan is for OLED.CPP to provide the master calls to orchestrate the OLEDINT.CPP (port of SSD1306) and
 */

#ifndef OLED_H_
#define OLED_H_

#ifdef __cplusplus
	extern "C" {
#define EXTERN
#else
#define EXTERN extern
#endif

	// Define the GPIO pincode values (used by the SPI library) for the D/C, RST, and CS pins
	// Other pins (SCLK, MOSI) are defined by SPI channel 0 block (master mode)
	// (Pincode values encode port (*32) and pin values in one int)
#define OLED_DC ((1<<5) | (18))
#define OLED_RST ((1<<5) | (19))
#define OLED_CS ((1<<5) | (20))

// these functions will return 0 if no device is found, nonzero otherwise
EXTERN int oled_init(void);
EXTERN int oled_begin(int reset);

// control various operation constants of the device
enum { DRAWMODE_DEFERRED, DRAWMODE_IMMEDIATE };
EXTERN void  oled_setDrawMode(int new_setting);
EXTERN int  oled_getDrawMode(void);
enum { TEXTWRAP_OFF, TEXTWRAP_ON };
EXTERN void  oled_setTextWrap(int new_setting);
EXTERN int  oled_getTextWrap(void);
enum { BLACK, WHITE, INVERT };
EXTERN void  oled_setGraphicsColor(int new_setting);
EXTERN int  oled_getGraphicsColor(void);
enum { NORMAL, INVERTED, NORMAL_OVL, INVERTED_OVL };
EXTERN void  oled_setTextColor(int new_setting);
EXTERN int  oled_getTextColor(void);
EXTERN void  oled_setTextSize(int new_setting);
EXTERN int  oled_getTextSize(void);
// graphics cursor functions
EXTERN void  oled_setCursor(int x, int y);
EXTERN int  oled_getCursorX(void);
EXTERN int  oled_getCursorY(void);
EXTERN int  oled_getWidth(void);
EXTERN int  oled_getHeight(void);
//EXTERN void  oled_set(int new_setting);
//EXTERN int  oled_get(void);

// drawing functions can operate in deferred mode (into RAM buffer) or immediate to device
// most operate using current cursor position
EXTERN void oled_clearDisplay(void);
EXTERN void oled_drawBitmap(const unsigned char* bitmap_array, unsigned short width_in_bits, unsigned short height_in_bits);
EXTERN void oled_println(const char* str);
EXTERN void oled_print(const char* str);
EXTERN void oled_drawCircleAt(int x, int y, int radius, int color, int filled);
EXTERN void oled_drawLineTo(int x, int y, int color); // from current cursor, which updates to "to" location afterwards

// effects are immediate mode only
EXTERN void oled_display(void); // immediate xfer graphics buffer to screen in any drawing mode
EXTERN void oled_invertDisplay(int i_0_is_normal_1_is_inverted);
EXTERN void oled_dim(int dim_1_or_0);
enum { SCROLL_STOP, SCROLL_LEFT, SCROLL_RIGHT, SCROLL_UP, SCROLL_DOWN, SCROLL_UPLEFT, SCROLL_DOWNLEFT, SCROLL_UPRIGHT, SCROLL_DOWNRIGHT };
EXTERN void oled_scroll(int scroll_type);

#ifdef __cplusplus
	}
#endif

#endif /* OLED_H_ */
