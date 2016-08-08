/*
 * led.c
 *
 *  Created on: Aug 6, 2016
 *      Author: Mike
 */
#include "lpc17xx_gpio.h"

/*-----------------------------------------------------------*/
// LPC1769 RGB LED CONTROL MODULE
/*-----------------------------------------------------------*/
void led2_init() {

	/*
	 * (1) The pincnfig module can be accessed from the pointer: LPC_PINCON
	 * The PINSEL1 can be accessed using: LPC_PINCON->PINSEL1
	 * Now configure the right bits in the pin selection regiser #1
	 *
	 * (2) The FIODIR register for gpio0 can be accessed as follows: LPC_GPIO0->FIODIR
	 * Now set the right bit in this register to set the gpio direction to output
	 */
	// red
	UNS_32 maskR = (0x03 << 12); // bits 12 and 13
	LPC_PINCON->PINSEL1 &= ~maskR; // 00 is GPIO function
	UNS_32 bitR = (0x01 << 22);
	LPC_GPIO0->FIODIR |= bitR; // set this bit to use pin as output
	// green
	UNS_32 maskG = (0x03 << 18); // bits 18 and 19
	LPC_PINCON->PINSEL7 &= ~maskG; // 00 is GPIO function
	UNS_32 bitG = (0x01 << 25);
	LPC_GPIO3->FIODIR |= bitG; // set this bit to use pin as output
	// blue
	UNS_32 maskB = (0x03 << 20); // bits 20 and 21
	LPC_PINCON->PINSEL7 &= ~maskB; // 00 is GPIO function
	UNS_32 bitB = (0x01 << 26);
	LPC_GPIO3->FIODIR |= bitB; // set this bit to use pin as output
}

void led2red(int onFlag) {

	/* The FIOSET register can be accessed as follows: LPC_GPIO0->FIOSET
	 * Now set the right bit to set the GPIO high (led2 on)
	 */
	UNS_32 bit = (0x01 << 22);
	if (onFlag)
		LPC_GPIO0->FIOCLR |= bit; // active low pin
	else
		LPC_GPIO0->FIOSET |= bit;
}

void led2green(int onFlag) {

	/* The FIOSET register can be accessed as follows: LPC_GPIO0->FIOSET
	 * Now set the right bit to set the GPIO high (led2 on)
	 */
	UNS_32 bit = (0x01 << 25);
	if (onFlag)
		LPC_GPIO3->FIOCLR |= bit; // active low pin
	else
		LPC_GPIO3->FIOSET |= bit;
}

void led2blue(int onFlag) {

	/* The FIOSET register can be accessed as follows: LPC_GPIO0->FIOSET
	 * Now set the right bit to set the GPIO high (led2 on)
	 */
	UNS_32 bit = (0x01 << 26);
	if (onFlag)
		LPC_GPIO3->FIOCLR |= bit; // active low pin
	else
		LPC_GPIO3->FIOSET |= bit;
}

#define RED_MASK (0x01)
#define GREEN_MASK (0x02)
#define BLUE_MASK (0x04)

static int getOnFlag(int code, int colorMask) {
	return ((code & colorMask) != 0) ? 1 : 0;
}

void led2_set(int code) {
	// the color code passed uses 1-bit color here, i.e. bit 0 is red, 1 is green, 2 is blue
	// thus we can test each bit and dispatch on or off for that color
	led2red(getOnFlag(code, RED_MASK));
	led2green(getOnFlag(code, GREEN_MASK));
	led2blue(getOnFlag(code, BLUE_MASK));
}

int led2color_black()	{ return 0 * RED_MASK + 0 * GREEN_MASK + 0 * BLUE_MASK; }
int led2color_red()		{ return 1 * RED_MASK + 0 * GREEN_MASK + 0 * BLUE_MASK; }
int led2color_green()	{ return 0 * RED_MASK + 1 * GREEN_MASK + 0 * BLUE_MASK; }
int led2color_blue()	{ return 0 * RED_MASK + 0 * GREEN_MASK + 1 * BLUE_MASK; }
int led2color_cyan()	{ return 0 * RED_MASK + 1 * GREEN_MASK + 1 * BLUE_MASK; }
int led2color_magenta()	{ return 1 * RED_MASK + 0 * GREEN_MASK + 1 * BLUE_MASK; }
int led2color_yellow()	{ return 1 * RED_MASK + 1 * GREEN_MASK + 0 * BLUE_MASK; }
int led2color_white()	{ return 1 * RED_MASK + 1 * GREEN_MASK + 1 * BLUE_MASK; }


const char* pccColorString(int color) {
	static const char* colors[] = {
			"--- = black",
			"R-- = red",
			"-G- = green",
			"RG- = yellow",
			"--B = blue",
			"R-B = magenta",
			"-GB = cyan",
			"RGB = white",
	};

	return colors[color & 0x07];
}


