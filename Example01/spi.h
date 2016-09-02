/*
 * spi.h
 *
 *  Created on: Aug 11, 2016
 *      Author: Mike
 */

#ifndef SPI_H_
#define SPI_H_

#ifdef __cplusplus
	extern "C" {
#define EXTERN
#else
#define EXTERN extern
#endif

#include "stdint.h"

#include "delay.h"

#define MSBFIRST (0)
#define SPI_MODE0 (0)

enum {
	SPI_SSEL_DEASSERT, SPI_SSEL_ASSERT
};
EXTERN void spi_pinWrite(int pincode, int val); // GPIO pin write (any SSEL, incl. P0.16 for SPI0 main device)
EXTERN int spi_pinRead(int pincode); // GPIO pin read
EXTERN void spi_pinMode(int pincode, int dir_in_is_zero); // set direction of GPIO pin

EXTERN int spi_init(int ssel_pincode);
EXTERN void *SPISettings(int clk, int msls, int mode);
EXTERN int spi_beginTransaction(void*);
EXTERN uint8_t spi_transfer(uint8_t x);
EXTERN void spi_endTransaction(void);
EXTERN int spi_getLastError(void);

#ifdef __cplusplus
	}
#endif


#endif /* SPI_H_ */
