/*
 * spi.h
 *
 *  Created on: Aug 11, 2016
 *      Author: Mike
 */

#ifndef SPI_H_
#define SPI_H_

#include "stdint.h"

#include "delay.h"

#define MSBFIRST (0)
#define SPI_MODE0 (0)

enum {
	SPI_SSEL_DEASSERT, SPI_SSEL_ASSERT
};
extern void  spi_pinWrite(int pincode, int val); // GPIO pin write (any SSEL, incl. P0.16 for SPI0 main device)
extern int  spi_pinRead(int pincode); // GPIO pin read
extern void spi_pinMode(int pincode, int dir_in_is_zero); // set direction of GPIO pin

extern int spi_init(int ssel_pincode);
extern void *SPISettings(int clk, int msls, int mode);
extern int spi_beginTransaction(void*);
extern uint8_t spi_transfer(uint8_t x);
extern void spi_endTransaction(void);
extern int spi_getLastError(void);


#endif /* SPI_H_ */
