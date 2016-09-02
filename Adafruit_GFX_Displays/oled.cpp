/*
 * oled.cpp
 *
 *  Created on: Sep 1, 2016
 *      Author: Mike Mehr
 */

#define ARDUINO 10000 // MLM - joking around but needed

#include "oled.h"
#include "Adafruit_SSD1306.h"

Adafruit_SSD1306 *pOLED = NULL;

// set up the pins on the LPC-1769 via the class into the SPI implementation (doesn't work for I2C for now)
// will return 0 if cannot instantiate the class, 1 if it's good to go
int oled_init(int DC, int RST, int CS, int reset) {
	if (!pOLED)
		pOLED = new Adafruit_SSD1306(DC, RST, CS);
	if (pOLED) {
		// call begin() here
		return 1;
	}
	return 0;
}

