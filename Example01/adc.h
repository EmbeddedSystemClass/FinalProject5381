/*
 * adc.h
 *
 *  Created on: Aug 6, 2016
 *      Author: Mike
 */

#ifndef ADC_H_
#define ADC_H_

// contents of ADC.H for eventual inclusion in header
#define ADC_MIN_INPUT (0)
#define ADC_MAX_INPUT (4096)
#define DAC_MIN_OUTPUT (0)
#define DAC_MAX_OUTPUT (1024)

extern void adc_init();
extern void dac_init(void);
extern void dac_out(int frac_10bit);
extern int adc_read_single(int channel);
extern int adcScale(int reading, int in_min, int in_max, int out_min, int out_max);
extern int frac2MV(int reading, int scale);

#endif /* ADC_H_ */
