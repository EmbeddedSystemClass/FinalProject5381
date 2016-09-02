/*
 * delay.h
 *
 *  Created on: Sep 2, 2016
 *      Author: Mike Mehr
 *
 *  Library routines to implement Arduino-style delay function on top of FreeRTOS/SysTick
 */

#ifndef DELAY_H_
#define DELAY_H_


extern void spi_delay(int msec_delay);
extern int spi_millis(void);


#endif /* DELAY_H_ */
