/*
 * led.h
 *
 *  Created on: Aug 6, 2016
 *      Author: Mike
 */

#ifndef LED_H_
#define LED_H_

// contents of LED.H
// basic setup
extern void led2_init();

// general color manipulation
extern void led2_set(int color_code);

// return a color debug string for a color code
extern const char* pccColorString(int color_code);

// routines to return color code constants by name
extern int led2color_black();
extern int led2color_red();
extern int led2color_green();
extern int led2color_blue();
extern int led2color_cyan();
extern int led2color_magenta();
extern int led2color_yellow();
extern int led2color_white();

// routines to manipulate the individual RGB LEDs inside the tricolor
// NOTE: This is mostly for add-ons such as the LEDPWM module
extern void led2red(int onFlag);
extern void led2green(int onFlag);
extern void led2blue(int onFlag);


#endif /* LED_H_ */
