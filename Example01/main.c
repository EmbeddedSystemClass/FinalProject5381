/*
    FreeRTOS V8.0.1 - Copyright (C) 2014 Real Time Engineers Ltd.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc17xx_gpio.h"
/* FreeRTOS+IO includes. */
//#include "FreeRTOS_IO.h"

/* Library includes. */
#include "lpc17xx_gpio.h"

/* Demo includes. */
#include "basic_io.h"
#include "stdio.h"

/* Used as a loop counter to create a very crude delay. */
#define mainDELAY_LOOP_COUNT		( 0xfffff )

/* The task functions. */
void vTask1( void *pvParameters );
void vTask2( void *pvParameters );

/*-----------------------------------------------------------*/
// ADC MODULE
/*-----------------------------------------------------------*/

#define MASK2BITS (0x03)

#define PINSEL_SET2BITMASK(regnum, shift, value) \
	LPC_PINCON->PINSEL##regnum &= ~(MASK2BITS << shift); \
	LPC_PINCON->PINSEL##regnum |= ((value & MASK2BITS) << shift)

#define MODESEL_SET2BITMASK(regnum, shift, value) \
	LPC_PINCON->PINMODE##regnum &= ~(MASK2BITS << shift); \
	LPC_PINCON->PINMODE##regnum |= ((value & MASK2BITS) << shift)

static void adc_init() {
	// Config process from User Manual:
	/*
	 * 29.1 Basic configuration
The ADC is configured using the following registers:
1. Power: In the PCONP register (Table 46), set the PCADC bit (12).
Remark: On reset, the ADC is disabled. To enable the ADC, first set the PCADC bit,
and then enable the ADC in the AD0CR register (bit PDN Table 532). To disable the
ADC, first clear the PDN bit, and then clear the PCADC bit.
2. Clock: In the PCLKSEL0 register (Table 40), select PCLK_ADC. To scale the clock for
the ADC, see bits CLKDIV in Table 532.
3. Pins: Enable ADC0 pins through PINSEL registers. Select the pin modes for the port
pins with ADC0 functions through the PINMODE registers (Section 8.5).
	 * */
	// Four-step process from homework slides
	// 1 - enable clock/power to ADC
	LPC_SC->PCONP |= (1 << 12);
	// WHAT ABOUT CLKSEL? It defaults to CCLK/4; we may eventually want to select CCLK/8, CCLK/2, or CCLK, but no need now
	// For future ref, the ADC config is on bits 25:24 of PCLKSEL0
	//  The default value is 00 (CCLK/4); 01 is CCLK/1, 10 is CCLK/2, 11 is CCLK/8
	// This is on pp.57-58 of the manual, sec.4.7.3, tables 40-42
	LPC_SC->PCLKSEL0 &= ~(0x03 << 24);
	// 2 - Configure all 8 channel pins as ADC (P0.2-3, .23-26, .30-31) using PINSEL0/1/3
	// From manual p.117 sec.8.5.1+, set xx to the following bit pairs:
	// 2a- set 10 to SEL0.5:4 for AD0.7 func on P0.2
	PINSEL_SET2BITMASK(0, 4, 0x2);
	// 2b- set 10 to SEL0.7:6 for AD0.6 func on P0.3
	PINSEL_SET2BITMASK(0, 6, 0x2);
	// 2c- set 01 to SEL1.15:14 for AD0.0 func on P0.23
	PINSEL_SET2BITMASK(1, 14, 0x1);
	// 2d- set 01 to SEL1.17:16 for AD0.1 func on P0.24
	PINSEL_SET2BITMASK(1, 16, 0x1);
	// 2e- set 01 to SEL1.19:18 for AD0.2 func on P0.25
	PINSEL_SET2BITMASK(1, 18, 0x1);
	// 2f- set 01 to SEL1.21:20 for AD0.3 func on P0.26
	PINSEL_SET2BITMASK(1, 20, 0x1);
	// 2g- set 11 to SEL3.29:28 for AD0.4 func on P1.30
	PINSEL_SET2BITMASK(3, 28, 0x3);
	// 2h- set 11 to SEL3.31:30 for AD0.5 func on P1.31
	PINSEL_SET2BITMASK(3, 30, 0x3);
	// 3 - Deselect any pull-up and pull-down resistors on those 8 pins
	// From manual p.121 sec.8.5.9+. always set 10 to select neither pull-up or pull-down resistors
	// 3a- set 10 to MODE0.5:4 for NPUPDR on P0.2
	MODESEL_SET2BITMASK(0, 4, 0x2);
	// 3b- set 10 to MODE0.7:6 for NPUPDR on P0.3
	MODESEL_SET2BITMASK(0, 6, 0x2);
	// 3c- set 10 to MODE1.15:14 for NPUPDR on P0.23
	MODESEL_SET2BITMASK(1, 14, 0x2);
	// 3d- set 10 to MODE1.17:16 for NPUPDR on P0.24
	MODESEL_SET2BITMASK(1, 16, 0x2);
	// 3e- set 10 to MODE1.19:18 for NPUPDR on P0.25
	MODESEL_SET2BITMASK(1, 18, 0x2);
	// 3f- set 10 to MODE1.21:20 for NPUPDR on P0.26
	MODESEL_SET2BITMASK(1, 20, 0x2);
	// 3g- set 10 to MODE3.29:28 for NPUPDR on P1.30
	MODESEL_SET2BITMASK(3, 28, 0x2);
	// 3h- set 10 to MODE3.31:30 for NPUPDR on P1.31
	MODESEL_SET2BITMASK(3, 30, 0x2);
	// 4 - Configure the ADC control register
	// PDN to operational (set bit 21) -- TIED TO PCONP BIT IN CERTAIN ORDER SET vs. CLR
	LPC_ADC->ADCR |= (1 << 21);
	// Falling edge triggered sampling (set bit 27)
	LPC_ADC->ADCR |= (1 << 27);
	// Disable burst mode (clear bit 16)
	LPC_ADC->ADCR &= ~(1 << 16);
	// Stop A/D conversion (during init) (clear bits 26:24)
	LPC_ADC->ADCR &= ~(0x07 << 24);
	// Provide clock divider X (sets bits 15:8 to X(8-bit LSB))
	int divider = 0;
	LPC_ADC->ADCR &= ~(0x0FF << 8);
	LPC_ADC->ADCR |= ((divider & 0x0FF) << 8);
	// Select channel you want (set bit 0 for channel 0)
	// shouldn't this be done in the conversion routine?
}

static void dac_init(void) {
	// NOTE: for testing purposes, I will use the DAC to output a voltage under program control
	// The use of AOUT conflicts with the use of AD0.3 input (channel 3)
	// Therefore, this should be called AFTER adc_init() so that the pinsel values aren't overwritten
	// As stated in the manual, there is no PCONP bit
	// Just set 10 to SEL1.21:20 for AOUT func on P0.26
	PINSEL_SET2BITMASK(1, 20, 0x2);
	// Specify no pull-up or pull-down resistors on P0.26
	// Same as 3f in adc_init()- set 10 to MODE1.21:20 for NPUPDR on P0.26
	MODESEL_SET2BITMASK(1, 20, 0x2);
}

/*
 * NOTES ON VALUES:
 * Values read or written to the A/D subsystem are scaled fractions of the range VRefN - VRefP.
 * The ADC will input 12-bit fraction values, but the DAC will only output the most significant 10-bit values of this fraction.
 * The voltages are nominally the power supply rails, 3.3v and 0v(ground), but are isolated.
 * So to get a particular voltage output on AOUT, we must apply this formula:
 * 		AOUT = ( (VALUE/1024) * (VREFP - VREFN)) + VREFN
 * For VREFN=0, we get AOUT = VALUE/1024 * VREFP. This assumes VALUE is from 0-1023.
 * Note that AOUT can be VREFN for VALUE=0, but never quite VREFP at VALUE=1023 (max), only one LSB short.
 *
 * Similarly, on input, the scale factor is 4096 instead of 1024, but we get the equation:
 * 		VALUE / 4096 = (AIN - VREFN) / (VREFP - VREFN), or if we set VREFN=0,
 * 		VALUE / 4096 = AIN / VREFP, where AIN is the input voltage on any one of the input ADC channels
 * 		The range of VALUE here is from 0 for AIN=VREFN, to 4095 for AIN=VREFP (minus one LSB). If AIN>=VREFP, we get overrun indicators.
 *
 * Note that the input values are scaled in the upper bits of a 16-bit word, for both ADC and DAC.
 * This means that the values we actually get or send have a maximum value of 0xFFFF (65535) as their scaling factor.
 * So we should adjust the above equations to use 65536 fraction scaling.
 *
 * */

// FOR TEST MODE, HOOK UP THE DAC AOUT PIN (port 0.26) to one of the input channel pins for testing (P0.2-3, P0.23-25, or P1.30-31)
static void dac_out(int frac_10bit) {
	portENTER_CRITICAL();
	UNS_32 dacrValue = LPC_DAC->DACR; // read current register values
	dacrValue &= ~(0x3FF << 6); // clear just the DACR bitfield
	dacrValue |= ((frac_10bit & 0x3FF) << 6); // set the new value to the bitfield
	LPC_DAC->DACR = dacrValue; // output the register with just the changes needed
	portEXIT_CRITICAL();
}

static int adc_read_single(int channel) {
	portENTER_CRITICAL();
	// outputs the 12-bit fractional value, unscaled (0-8191) for the given channel pin number (0-7)
	int chan = channel & 0x07; // use only lower 3 bits of channel number
	// seven steps in the homework:
	// 1- Disable all ADC channels
	LPC_ADC->ADCR &= ~(0xFF << 0);
	// 2- Enable just the channel in question
	LPC_ADC->ADCR |= (0x01 << chan);
	// 3- Start A/D conversion immediately
	LPC_ADC->ADCR |= (0x01 << 24); // assumes bits 25-26 are 0 as originally set
	// 4- Input loop:
	UNS_32 done, gdrValue;
	do {
		// 4a- read GDR global data register
		gdrValue = LPC_ADC->ADGDR;
		// 4b- Check if conversion is done
		// 4c- Break only if done
		// 4d- Else loop back
		done = gdrValue & (1 << 31);
	} while (!done);
	// 5- Stop A/D conversion
	LPC_ADC->ADCR &= ~(0x01 << 24); // assumes bits 25-26 are 0 as originally set
	// 6- Check for overrun/lost data
	UNS_32 overrun = gdrValue & (1 << 30);
	// 7- Return 0 if OVR, else return data from GDR
	int output = 0;
	if (!overrun)
		output = (gdrValue >> 4) & 0xFFF;
	portEXIT_CRITICAL();
	return output;
}

static double fround(double input) {
	if (input < 0.0)
		return (int)(input - 0.5);
	return (int)(input + 0.5);
}

#define VREFP_MV (3300)
#define VREFN_MV (0)
static int frac2MV(int reading, int scale) {
	double fraction = reading / (double)scale;
	double output =  VREFN_MV + (VREFP_MV - VREFN_MV) * (fraction);
	return fround(output);
}

/*-----------------------------------------------------------*/
static int channelToRead = 3; // ADC input channel 0-7
int main( void )
{
	/* Init the semi-hosting. */
	printf( "\n" );
	adc_init();
	dac_init();
	dac_out(1023);
//	int in = adc_read_single(0);
//	vPrintStringAndNumber("Read a value = ", in);

	/* Create one of the two tasks. */
	xTaskCreate(	vTask1,		/* Pointer to the function that implements the task. */
					"Task 1",	/* Text name for the task.  This is to facilitate debugging only. */
					240,		/* Stack depth in words. */
					channelToRead,		/* Pass the AIN channel # in the task parameter. */
					1,			/* This task will run at priority 1. */
					NULL );		/* We are not using the task handle. */

	xTaskCreate(	vTask2,		/* Pointer to the function that implements the task. */
					"Task 2",	/* Text name for the task.  This is to facilitate debugging only. */
					240,		/* Stack depth in words. */
					NULL,		/* We are not using the task parameter. */
					1,			/* This task will run at priority X. */
					NULL );		/* We are not using the task handle. */


	/* Start the scheduler so our tasks start executing. */
	vTaskStartScheduler();

	/* If all is well we will never reach here as the scheduler will now be
	running.  If we do reach here then it is likely that there was insufficient
	heap available for the idle task to be created. */
	for( ;; );
	return 0;
}

/*-----------------------------------------------------------*/
void vTask1( void *pvParameters )
{
int channel = (int)pvParameters;
//	adc_init();
	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		// read the current value of the ADC
		int value = adc_read_single(channel);
		int mv = frac2MV(value, 4096);

		/* Print out the name of this task and the value read. */
		printf( "Task 1: ADC reading = %d/4096, input (mv) = %d\n", value, mv );

		// add any optional delay here - otherwise just read and print in a tight loop
	}
}

void vTask2( void *pvParameters )
{
unsigned int ul = 0;
//	dac_init(); // TEST MODE ONLY
	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{

		/* Delay for a period. */
		vTaskDelay(1000 / configTICK_RATE_HZ);

		/* Print out the name of this task. */
		int value = (ul & 0x3ff);
		int mv = frac2MV(value, 1024);
		printf( "***>>> Task 2: DAC setting = %d/1024 = %d/4096, output (mv) = %d\n", value, value << 2, mv );

		// output an ever-increasing value to the DAC (will be converted into a 10-bit fraction internally by bit masking)
		// This should output a stair-step wave on AOUT at a frequency set by the delay value
		dac_out(value);
		ul += 64;


	}
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* This function will only be called if an API call to create a task, queue
	or semaphore fails because there is too little heap RAM remaining. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will only be called if a task overflows its stack.  Note
	that stack overflow checking does slow down the context switch
	implementation. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* This example does not use the idle hook to perform any processing. */
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This example does not use the tick hook to perform any processing. */
}
