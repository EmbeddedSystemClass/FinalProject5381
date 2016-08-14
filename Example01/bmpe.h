/*
 * bmpe.h
 *
 *  Created on: Aug 11, 2016
 *      Author: Mike
 */

#ifndef BMPE_H_
#define BMPE_H_

 // public:
	enum {
		BMPE_NONE,
		BMPE_BMPTYPE,
		BMPE_BMETYPE
	};
    extern int  bmpe_init(void);

    extern float bmpe_readTemperature(void);
    extern float bmpe_readPressure(void);
    extern int bmpe_hasHumidity(void);
    extern float bmpe_readHumidity(void);

    // give relative altitude to reference which starts as one standard atmosphere at standard temp
    // see https://en.wikipedia.org/wiki/Atmospheric_pressure
    extern float bmpe_readAltitude();
    extern void bmpe_readReferencePressure();


#endif /* BMPE_H_ */
