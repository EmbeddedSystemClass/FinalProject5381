/*
 * spi.c
 *
 *  Created on: Aug 11, 2016
 *      Author: Mike
 */
#include "spi.h"

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_spi.h"

/*
 * This module is intended to be a thin layer between the Adafruit Arduino libraries SPI implementation and that from the NXP LPC17xx CMSIS driver library.
 * All attempts will be made for this to be a direct translation, but it may get ugly (!).
 *
 * I'll use this space for notes as I learn how to do this as I go.
 * Wish me luck!
 *
 * -- Mike Mehr, Aug.12, 2016
 *
 *
///// START MANUAL EXCERPT
 *
UM10360 All information provided in this document is subject to legal disclaimers. © NXP B.V. 2014. All rights reserved.
User manual Rev. 3.1 — 2 April 2014 414 of 849
NXP Semiconductors UM10360
Chapter 17: LPC176x/5x SPI

 * 17.6.2 Master operation

The following sequence can be followed to set up the SPI prior to its first use as a master.
This is typically done during program initialization.
1. Set the SPI Clock Counter Register to the desired clock rate.
2. Set the SPI Control Register to the desired settings for master mode.

The following sequence describes how one should process a data transfer with the SPI
block when it is set up to be the master. This process assumes that any prior data transfer
has already completed.
1. Optionally, verify the SPI setup before starting the transfer.
2. Write the data to transmitted to the SPI Data Register. This write starts the SPI data
transfer.
3. Wait for the SPIF bit in the SPI Status Register to be set to 1. The SPIF bit will be set
after the last cycle of the SPI data transfer.
4. Read the SPI Status Register.
5. Read the received data from the SPI Data Register (optional).
6. Go to step 2 if more data is to be transmitted.
Note: A read or write of the SPI Data Register is required in order to clear the SPIF status
bit. Therefore, if the optional read of the SPI Data Register does not take place, a write to
this register is required in order to clear the SPIF status bit.

///// END MANUAL EXCERPT

I would add that the sequence involves asserting SSEL separately, because the rest of the wires can be shared with multiple devices, but NOT the SSEL pin.
Any GPIO pin could be used for another SSEL. The main one also serves as a GPIO pin, so it has the usual setup as well (PINSEL, PINMODE, FIODIR).

Also, the functions specified in the LPCOpen periph_spi app should be consulted for a more definitive approach, not one translated from Arduino necessarily.

*/

// ARDUINO PORT: simple implementation of GPIO functions to set direction, read, and write a pin
static void getPortAndPin(int pincode, int* port, int* pin) {
	// Arduino only uses first 32 pins, but 1769 has 64 pins that we can code
	// So, for example, pincode=49 => port 3, pin 1 (0-based)
	*port = pincode / 32; // integer division
	*pin = pincode % 32;
}

// this is provided on Arduino for setting the FIODIR bits
void spi_pinMode(int pincode, int val)
{
	int port, pin;
	getPortAndPin(pincode, &port, &pin);
	FIO_SetDir(port, (1 << pin), val);
}

// GPIO pin write (can be used for any SSEL, incl. P0.16 for SPI0 main device)
void  spi_pinWrite(int pincode, int val)
{
	int port, pin;
	getPortAndPin(pincode, &port, &pin);
	if (val)
		GPIO_SetValue(port, pin);
	else
		GPIO_ClearValue(port, pin);
	return;
}

int spi_pinRead(int pincode)
{
	int port, pin, val=0;
	getPortAndPin(pincode, &port, &pin);
	val = GPIO_ReadValue(port);
	val = (val & (1 << pin)) ? 1 : 0; // test one bit of result
	return val;
}

// this structure controls the basic SPI operational settings (CPHA, CPOL, Mode, DataOrder, ClockRate/Hz)
SPI_CFG_Type currentSettings;

int spi_init() {
	// set up defaults (MASTER mode)
	SPI_ConfigStructInit(&currentSettings);
	// enable clock/power to SPI

	// configure pin select functions
	// P0.15 is SCLK, P0.16 is SSEL, P0.17 is MISO, P0.18 is MOSI
	PINSEL_CFG_Type pincfg;
	pincfg.Portnum = PINSEL_PORT_0;
	pincfg.Funcnum = PINSEL_FUNC_3; // SPI is func 3=0b11
	pincfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	pincfg.OpenDrain = PINSEL_PINMODE_NORMAL;

	pincfg.Pinnum = PINSEL_PIN_15;
	PINSEL_ConfigPin(&pincfg);
	pincfg.Pinnum = PINSEL_PIN_16;
	PINSEL_ConfigPin(&pincfg);
	pincfg.Pinnum = PINSEL_PIN_17;
	PINSEL_ConfigPin(&pincfg);
	pincfg.Pinnum = PINSEL_PIN_18;
	PINSEL_ConfigPin(&pincfg);

	// direction (input/output) should be specified
	// and any output presets should be defined before that
	// this is done in the Arduino code port

	// and flush the data lines

	return 0;
}

void *SPISettings(int clk, int msls, int mode) {
	currentSettings.Mode = SPI_MASTER_MODE;
	currentSettings.Databit = SPI_DATABIT_8;
	currentSettings.ClockRate = clk;
	currentSettings.DataOrder = msls? SPI_DATA_MSB_FIRST: SPI_DATA_LSB_FIRST;
	currentSettings.CPHA = (mode & 0b001)? SPI_CPHA_SECOND: SPI_CPHA_FIRST;
	currentSettings.CPOL = (mode & 0b010)? SPI_CPOL_LO: SPI_CPOL_HI;
	return (void *) &currentSettings;
}

int spi_beginTransaction(void* params) {
	SPI_Init(LPC_SPI, params);
	return 0;
}

int lastError = 0;

int spi_getLastError() {
	int le = lastError;
	lastError = 0;
	return le;
}

uint8_t spi_transfer(uint8_t x) {
	// the intent here is to swap one byte of data with the slave altho SPI is capable of multibyte transfers
	// polling mode will be used (no interrupts)
	uint8_t result;
	SPI_DATA_SETUP_Type xf_setup;
	xf_setup.tx_data = &x;
	xf_setup.rx_data = &result;
	xf_setup.length = 1;
	int numReceived = SPI_ReadWrite(LPC_SPI, &xf_setup, SPI_TRANSFER_POLLING);
	// the return value could be
	//   1 = AOK, byte was received
	//   0 = no bytes received OR interrupt mode
	//  -1 = any kind of error
	if (numReceived < 1) {
		// deal with errors here
		// accumulate in lastError buffer, in case we do multiple calls before we notice
		lastError |= xf_setup.status;
		// only a mode failure causes action: we need to re-send the port params in that case
		if (xf_setup.status & SPI_SPSR_MODF) {
			SPI_Init(LPC_SPI, &currentSettings);
		}
	}
	return result;
}

void spi_endTransaction(void) {
	return;
}
