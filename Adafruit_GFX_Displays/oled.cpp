/*
 * oled.cpp
 *
 *  Created on: Sep 1, 2016
 *      Author: Mike Mehr
 */

#define ARDUINO 10000 // MLM - joking around but needed

#include "oled.h"
#include "Adafruit_SSD1306.h"

Adafruit_SSD1306 OLED(OLED_DC, OLED_RST, OLED_CS);

// set up the pins on the LPC-1769 via the class into the SPI implementation (doesn't work for I2C for now)
// will return 0 if cannot instantiate the class, 1 if it's good to go
int oled_init(int DC, int RST, int CS, int reset) {
	// perform the reset if asked, return result
	return 0;
}

