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

EXTERN int oled_init(void);
EXTERN int oled_begin(int reset);

EXTERN void oled_clearDisplay(void);
EXTERN void oled_invertDisplay(int i_0_is_normal_1_is_inverted);
EXTERN void oled_display();

EXTERN void oled_dim(int dim_1_or_0);

#ifdef __cplusplus
	}
#endif

#endif /* OLED_H_ */
