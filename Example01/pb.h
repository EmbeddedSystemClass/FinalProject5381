/*
 * pb.h
 *
 *  Created on: Aug 6, 2016
 *      Author: Mike
 */

#ifndef PB_H_
#define PB_H_

// set up the GPIO system for PB input use
extern void pb_init(int pincode);

// Debounce the GPIO HW pin whose code is given
// pass a state variable to use for data accumulation in the 2nd param (as a ptr)
// returns +1 if stable rising edge detected, -1 if stable falling edge detected, 0 otherwise
extern int pb_detect(int pincode, int * pState);

// Debounce immediate stream of bit values
// each bit_value should be 0 or nonzero (treated as 1)
// pass a state variable to use for data accumulation in the 2nd param (as a ptr)
// returns +1 if stable rising edge detected, -1 if stable falling edge detected, 0 otherwise
extern int pb_detect_immediate(int bit_value, int * pState);


#endif /* PB_H_ */
