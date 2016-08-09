/*
 * simple7.c
 *
 *  Created on: Aug 8, 2016
 *      Author: Mike
 */

#include "lpc17xx_gpio.h"

/*-----------------------------------------------------------*/
// SIMPLE ONE-DIGIT SEVEN-SEGMENT DISPLAY CONTROL MODULE
/*-----------------------------------------------------------*/

/*
 * This module controls GPIO Port 2 Pins 2.7-2.0 to drive eight segments of a 7-segment LED display.
 *
 * I have set the output wiring so that the bits of the port correspond to the output segments in alphabetical order, MSB to LSB, with DP as the MSB.
 * =================================================
 * |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
 * -------------------------------------------------
 * |  DP |  A  |  B  |  C  |  D  |  E  |  F  |  G  |
 * =================================================
 *
 * A simple font table is provided for standard hex display (0-15 map to decimal numbers and letters A-F).
 * An alternate font table is provided for testing.
 *
 * */
typedef uint8_t CharBitPattern;

void s7_init() {

	/*
	 * For GPIO functions of Port 2.0-2.7, write repeating pattern 0b00 (0x0000) to the lower 16 bits of PINSEL4
	 * For no pull-up or pull-down resistors, write repeating pattern 0b10 (0xAAAA) to the lower 16 bits of PINMODE4
	 * For output directionality, set 0b1 to each of the lower 8 bits (0xFF) of FIO2DIR
	 */
	uint32_t temp;
	// set 8 function pins to GPIO function
	LPC_PINCON->PINSEL4 = ~0xFFFF; // 00 is GPIO function
	// set 8 mode pins to have no pull-up or pull-down resistors
	temp = LPC_PINCON->PINMODE4 &= ~0xFFFF; // input current register, masked to clear LSW (15:0)
	LPC_PINCON->PINMODE4 = temp | 0xAAAA; // output by OR-ing with LSW 8 shifted 0b10 pairs into LSW (15:0)
	// send 1's to lower byte of FIO2DIR to select port outputs
	LPC_GPIO2->FIODIR0 = 0x00FF;
	// output all zeroes to bottom byte of FIO2MASK to make sure outputs go out (default but reset any other usage)
	LPC_GPIO2->FIOMASK0 = 0;
}

void s7_write(CharBitPattern raw_output) {

	/* Use the FIOMASK register to ignore the upper 3 bytes and write to the lower.
	 * Then just output the raw output directly.
	 */
	LPC_GPIO2->FIOPIN0 = raw_output;
}

static CharBitPattern testFont[] = {
		0b00000000, // 0x00: all off
		0b00000001, // 0x01-07: test pattern singles (0x01 = '-')
		0b00000010,
		0b00000100,
		0b00001000, // 0x04 = '_'
		0b00010000,
		0b00100000,
		0b01000000, // 0x07 = single top line
		0b10000000, // 0x08: DP test
		0b01000001, // 0x0a-10: test pattern pairs
		0b00100010,
		0b00010100,
		0b00001000,
		0b00010100,
		0b00100010,
		0b01000001,
		0b10101010, // 0x11-12: test pattern alternate quads
		0b01010101,
		0b01100010, // 0x13: top hat
		0b00011100, // 0x14: cup
		0b01100011, // 0x15: degrees
		0b00011101, // 0x16: little o
		0b00100010, // 0x17: parallel top
		0b00010100, // 0x18: parallel btm
		0b00110110, // 0x19: parallel long
		0b11111111, // 0x14: all on
};

#define TEST_FONT_SIZE ( sizeof(testFont)/sizeof(testFont[0]) )

/*
Truth table for 7-segment output.
Taken from here: http://www.petervis.com/electronics%20guides/7%20Segment%20LED%20Display/7%20Segment%20LED%20Display.html
    A	B	C	D	E	F	G
0	×	×	×	×	×	×
1		×	×
2	×	×		×	×		×
3	×	×	×	×			×
4		×	×			×	×
5	×		×	×		×	×
6	×		×	×	×	×	×
7	×	×	×
8	×	×	×	×	×	×	×
9	×	×	×	×		×	×
A	×	×	×		×	×	×
b			×	×	×	×	×
C	×			×	×	×
d		×	×	×	×		×
E	×			×	×	×	×
F	×				×	×	×
*/
static CharBitPattern hexFont[] = {
		0b01111110, // 0
		0b00110000, // 1
		0b01101101, // 2
		0b01111001, // 3
		0b00110011, // 4
		0b01011011, // 5
		0b01011111, // 6
		0b01110000, // 7
		0b01111111, // 8
		0b01111011, // 9
		0b01110111, // A
		0b00011111, // b
		0b01001110, // C
		0b00111101, // d
		0b01001111, // E
		0b01000111, // F
};

#define HEX_FONT_SIZE ( sizeof(testFont)/sizeof(testFont[0]) )

// return the given test pattern, else blank pattern
CharBitPattern getTestDigitOutput(int test) {
	int index = test;
	if (test < 0) index = 0;
	if (test >= TEST_FONT_SIZE) index = 0;
	CharBitPattern retval = testFont[index];
	return retval;
}

// translate a binary number to bit pattern via hex font
// OPT: if useDP is != 0, DP is turned on as well
CharBitPattern getHexDigitOutput(int byteVal, int useDP) {
	CharBitPattern retval;
	int index = byteVal & 0x0F;
	retval = hexFont[index];
	if (useDP)
		retval |= testFont[8];
	return retval;
}

// translate an ASCII character to bit pattern via hex font
// accepts UC and LC letters A-F, digits 0-9, and decimal points "." and ","
// all other characters result in a blank
// OPT: if useDP is != 0, DP is turned on as well
CharBitPattern getAsciiDigitOutput(char asciiChar, int useDP) {
	CharBitPattern retval = testFont[0];
	switch (asciiChar) {
	case '0':
	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
	case '9':
		retval = hexFont[0 + asciiChar];
		break;
	case 'a':
	case 'A':
		retval = hexFont[0x0a];
		break;
	case 'b':
	case 'B':
		retval = hexFont[0x0b];
		break;
	case 'c':
	case 'C':
		retval = hexFont[0x0c];
		break;
	case 'd':
	case 'D':
		retval = hexFont[0x0d];
		break;
	case 'e':
	case 'E':
		retval = hexFont[0x0e];
		break;
	case 'f':
	case 'F':
		retval = hexFont[0x0f];
		break;
	case '.':
	case ',':
		retval = testFont[0x08];
		break;
	default:
		break;
	}
	return getHexDigitOutput(retval, useDP);
}

// get number of test patterns
int s7_getNumberOfTestPatterns()
{
	return TEST_FONT_SIZE;
}

// write test pattern with given index (0-#tests)
void s7_writeTest(int value)
{
	CharBitPattern pattern = getTestDigitOutput(value);
	s7_write(pattern);
}

// write a digit (decimal or hex) with binary value given (0-15) and optional decimal point
void s7_writeBinary(int value, int useDP)
{
	CharBitPattern pattern = getHexDigitOutput(value, useDP);
	s7_write(pattern);
}

// ADDED - overflow feature; turns DP on if out of range 0-15 (and displays value modulo that range)
int s7_getDataRange(void) { return 16; }
void s7_writeBinaryDP(int value)
{
	int RANGE = s7_getDataRange();
	int useDPint = (value >= RANGE)? 1: 0;
	int valueInt = value % RANGE;
	s7_writeBinary(valueInt, useDPint);
}

// write an ASCII character with given value and optional decimal point
// valid character inputs are digits, letters A-F or a-f, comma, period
void s7_writeAscii(char value, int useDP)
{
	CharBitPattern pattern = getAsciiDigitOutput(value, useDP);
	s7_write(pattern);
}
