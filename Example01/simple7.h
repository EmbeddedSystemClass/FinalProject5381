/*
 * simple7.h
 *
 *  Created on: Aug 8, 2016
 *      Author: Mike
 */

#ifndef SIMPLE7_H_
#define SIMPLE7_H_

// set up the port pins (port 2.0-2.7)
void s7_init();

// write a digit (decimal or hex) with binary value given (0-15) and optional decimal point
void s7_writeBinary(int value, int useDP);

// ADDED - overflow feature; turns DP on if out of range 0-15 (and displays value modulo that range)
void s7_writeBinaryDP(int value);

// write an ASCII character with given value and optional decimal point
// valid character inputs are digits, letters A-F or a-f, comma, period
void s7_writeAscii(char value, int useDP);

// TEST PATTERN INTERFACE
// get number of test patterns
int s7_getNumberOfTestPatterns();

// write test pattern with given index (0-#tests)
void s7_writeTest(int value);

#endif /* SIMPLE7_H_ */
