/*
 * pb.c
 *
 *  Created on: Aug 6, 2016
 *      Author: Mike Mehr
 * GPIO read and debounce module
 *
 *  NOTES:
 *  I am using GPIO P0.9 for the pushbutton
 *  I also will use GPIO P0.0 for the PIR sensor
 *  This is brought out to pin 5 of J2 master 54-pin connector on the LPC-1769
 *  The input circuit is set up to wire the switch between VDD (3V3 or 3.3v) and the input pin.
 *  To save use of an outboard resistor, the input pull-down resistor function is used.
 *  The data sheet shows this to be a nearly-linear current source of around 50-60ua. Anyway, it works.
 *  The surprisingly-simple debouncer below was nabbed and modified from an online paper on debouncing.
 *   See article on debouncing at: http://www.eng.utah.edu/~cs5780/debouncing.pdf
 *  It was highly recommended in there to avoid hooking the switch up to an Interrupt, saying that this is much more sensitive to errors.
 *  Instead, I have created a simple sampling debouncer that can be called from a timer interrupt or delay task sample loop.
 *
 */
#include "lpc17xx_gpio.h"

void pb_init(int pincode)
{
	int selcode = 2 * pincode;
	//configure pin for gpio functionality (00 => PINSEL0.19:18)
	LPC_PINCON->PINSEL0 &= (~(0b11 << selcode));
	//configure mode pin for pulldown resistor (11 => PINMODE0.19:18)
	LPC_PINCON->PINMODE0 |= ((0b11 << selcode));
	//configure gpio as input (set the right bit to *zero*)
    LPC_GPIO0->FIODIR &= (~(1 << pincode));
    // configure gpio mask register to allow reads on pin 9 (set the right bit to zero)
    LPC_GPIO0->FIOMASK &= (~(1 << pincode));
}

int RawKeyPressed(int pincode) {
	if (LPC_GPIO0->FIOPIN & (1<<pincode))
		return 1;
	return 0;
}

// Service routine to be called by a timer interrupt or regular system function or task
/*
 * I modified the routine DebounceSwitch2, since it was only detecting leading edges.
 * The modified routine returns +1 on a rising edge (one zero followed by N-1 stable 1's)
 * It returns -1 if detecting a trailing edge (one 1 followed by N-1 stable 0's)
 *
 * Since the bit states are shifted in to the left as negated keypress values (see Discussion),
 *   a zero read from the pin comes in as a 1 into the shift register (State var).
 *   Oldest bits are towards the MS end of State.
 *   The three MS bits are set to 1 (0xe000) to shorten the shift register by 3 bits.
 * The use of 0xe000, 0xf000, and 0xefff are to detect the N-bit sequences, where N=13 for 0xe000.
 *   To use a shorter sequence (faster time, less noise cancellation), use for example N:
 *   	N=13	0xe000	0xf000	0xefff
 *   	N=12	0xf000	0xf800	0xf7ff
 *   	N=11	0xf800	0xfc00	0xfbff
 *
 *  DISCUSSION OF DESIGN
 *
 *  I believe that bit inversion (!RawKeyPressed()) to allow easy use of the OR operation to include the new bit.
 *  ORing the upper bits creates a don't-care state for them, effectively shortening the delay.
 *  Combined with the edge detectors, this allows an easy way of getting the input to delay N samples for stability.
 *
 *  To get a reasonable response time of 50-60 msec, the number N and the call rate should be set appropriately.
 *  For example, if this is called every 5 msec, it will take 65msec to get a stable detection.
 * */
typedef struct {
	int N;
	uint16_t dontCare;
	uint16_t riseDetect;
	uint16_t fallDetect;
} EdgeParam;

static EdgeParam edgeParams[] = {
		 {   	13,	0b1110000000000000,	0b1111000000000000,	0b1110111111111111 }, //0xe000,	0xf000,	0xefff },
		 {   	12,	0b1111000000000000,	0b1111100000000000,	0b1111011111111111 }, //0xf000,	0xf800,	0xf7ff },
		 {   	11,	0b1111100000000000,	0b1111110000000000,	0b1111101111111111 }, //0xf800,	0xfc00,	0xfbff },
		 {   	10,	0b1111110000000000,	0b1111111000000000,	0b1111110111111111 }, //0xfc00,	0xfe00,	0xfdff },
		 {   	 9,	0b1111111000000000,	0b1111111100000000,	0b1111111011111111 }, //0xfe00,	0xff00,	0xfeff },
		 {   	 8,	0b1111111100000000,	0b1111111110000000,	0b1111111101111111 }, //0xff00,	0xff80,	0xff7f },
		 { 0, 0, 0, 0 } // must be last value
};

EdgeParam* lookupParams(int N) {
	EdgeParam* output;
	for (output = edgeParams; output->N != 0; ++output) {
		if (output->N == N)
			break;
	}
	return output; // check - it could be not found! make sure result->N is what you wanted
}

int pb_detect_internal(int pincode, int *pStateRaw, int value, int paramIndex)
{
//static uint16_t State = 0; // Current debounce status
	EdgeParam* pE = &edgeParams[paramIndex];
	uint16_t* pState = (void *)pStateRaw; // playing fast and loose with the pointers! hopefully the cast is to the LSWord (16-bit) within the int (prob.32 bit for LPC1769)
	int bit = (value<0) ? RawKeyPressed(pincode) : (value==0? 0: 1);
	*pState=(*pState<<1) | !bit | pE->dontCare;
	if (*pState==pE->riseDetect) return 1;
	if (*pState==pE->fallDetect) return -1;
	return 0;
}

int pb_detect(int pincode, int * pStateRaw) {
	return pb_detect_internal(pincode, pStateRaw, -1, 0); // use N=13
}

int pb_detect_immediate(int bit_value, int * pStateRaw) {
	return pb_detect_internal(-1, pStateRaw, bit_value, 3); // use N=10
}

