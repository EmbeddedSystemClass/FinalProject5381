/*
 * simple7.c
 *
 *  Created on: Aug 8, 2016
 *      Author: Mike
 */

//#include "lpc_types.h"
#include "lpc17xx_pinsel.h"
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
 * UPDATE: An alternate wiring scheme is provided on Port 1 Pins 1.25-1.18. Motivation was that my port 2 functionality was broken during header soldering (perhaps).
 *
 * P2 WIRING TABLE:
 * DP	P2.7	J2p42
 * A	P2.6	J2p43
 * B	P2.5	J2p44
 * C	P2.4	J2p45
 * D	P2.6	J2p46
 * E	P2.2	J2p47
 * F	P2.1	J2p48
 * G	P2.0	J2p49
 *
 * P1 WIRING TABLE:
 * DP	P1.25	PAD 11
 * A	P2.24	PAD 17
 * B	P2.23	PAD  6
 * C	P2.22	PAD 12
 * D	P2.21	PAD 18
 * E	P2.20	PAD  7
 * F	P2.19	PAD 13
 * G	P2.18	PAD 19
 * NOTE: Due to placement of the pads in a square array out of numerical order, the segment bits are adjacent as follows.
 *
 * Looking down from above at the board edge, we see:
 * PAD1 (row by itself)
 * PAD2 	PAD3 	PAD4 	PAD5 	PAD6/B 	PAD7/E
 * PAD8 	PAD9 	PAD10	PAD11/.	PAD12/C	PAD13/F
 * PAD14	PAD15	PAD16	PAD17/A	PAD18/D	PAD19/G
 *
 * Arrangements around the display are:
 *
 *
 * */
typedef uint8_t CharBitPattern;

//#define PORT2_WIRING // uncomment this line to use Port 2.7:0
#define PORT1_WIRING // uncomment this line to use Port 1.25:18

#ifdef PORT2_WIRING
#define S7PORT (2)
#define S7LSPIN (0)
#else // PORT1_WIRING
#define S7PORT (1)
#define S7LSPIN (18)
#endif
#define S7MSPIN (S7LSPIN + 7)

void s7_init() {

	/*
	 * Configure using the CMSIS library functions PINSEL_* and FIO_*
	 * Configuration deals with Pinmode (the pullup/pulldown resistors), open drain, and functions 00-11
	 * We will need GPIO (func.0) and no resistors or open drain pins.
	 * In addition, we need to set the FIOMASK bits to 0 to allow output through the FIOPIN register,
	 *   and the FIODIR bits to 1 to the output direction.
	 *
	 */

	PINSEL_CFG_Type config;
	config.OpenDrain = PINSEL_PINMODE_NORMAL;
	config.Pinmode = PINSEL_PINMODE_TRISTATE;
	config.Funcnum = PINSEL_FUNC_0; // GPIO
	int port = S7PORT;
	config.Portnum = port;
	for (int i=0; i<8; ++i) {
		int pin = S7LSPIN + i; // assumes pins are adjacent (for now)
		// OPTIONAL: this can be scanned in from an array of pin codes (port * 32 + pin)
		// so the file really can support any 8 GPIO pins for segment output.
		config.Pinnum = pin;
		PINSEL_ConfigPin(&config);
		// set the output mask bit to 0 to allow outputs through
		FIO_SetMask(port, (1<<pin), 0);
		// set the direction bit to 1 for output
		FIO_SetDir(port, (1<<pin), 1);
	}
}

void s7_write(CharBitPattern raw_output) {

	/* Use the FIOMASK register to ignore the upper 3 bytes and write to the lower.
	 * Then just output the raw output directly, shifting over by the number of the LS pin.
	 * This is done by writing directly to the FIOPIN register.
	 *
	 * NOTE: The CMSIS library does not seem to provide this functionality directly.
	 */
	// currently this only supports ports 1 and 2; the CMSIS library has a static function supporting ports 0-4
	LPC_GPIO_TypeDef * pGPIO = (S7PORT == 2) ? LPC_GPIO2 : LPC_GPIO1;
	pGPIO->FIOPIN = (raw_output << S7LSPIN);
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
		retval = hexFont[asciiChar - '0'];
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
	if (useDP)
		retval |= testFont[8];
	return retval;
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
