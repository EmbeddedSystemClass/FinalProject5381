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
    extern void bmpe_measureForcedMode();

    extern float bmpe_readTemperature(void);
    extern float bmpe_readPressure(void);
    extern int bmpe_hasHumidity(void);
    extern float bmpe_readHumidity(void);

    // give relative altitude to reference which starts as one standard atmosphere at standard temp
    // see https://en.wikipedia.org/wiki/Atmospheric_pressure
    extern float bmpe_readAltitude();
    extern void bmpe_setReferencePressure();

    // PREFERRED METHOD OF READING SENSORS IN NORMAL MODES (filtering, etc.)
    // Since this only sends one read command, it will get all values associated with the same sample.
    // Individual reads of Temp, Pressure, and/or Humidity are not guaranteed to come from the same sample
    extern void bmpe_readAllSensors(float* pfTemperature, float* pfPressure, float* pfAltitude, float* pfHumidity);

#endif /* BMPE_H_ */
