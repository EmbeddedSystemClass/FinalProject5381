/*
 * bmpe.c
 *
 *  Created on: Aug 11, 2016
 *      Author: Mike Mehr, porting and adaptation to LPC-1769
 *      Original author: Adafruit Industries (see below)
 *
 *  This is a library for controlling either the Bosch BMP-280 or BME-280 environmental sensors.
 *  The BMP-280 does temperature and pressure/altitude. The BME-280 adds humidity but costs twice as much (summer 2016).
 *
 *  The devices URLs are here:
 *
 *  This module is a direct port of the Adafruit libraries, with translation to C and some name changes to allow LPC-1769 operation.
 *
 * NEW FEATURES:
 *  It figures out at runtime if it is being asked to connect to a BMP or BME and acts accordingly.
 *  (Humidity functions on the BMP only return 0.)
 *
 */

/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor
  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650
  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
/***************************************************************************
  This is a library for the BMP280 pressure sensor
  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651
  These sensors use I2C to communicate, 2 pins are required to interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

//#include "lpc17xx_gpio.h"
#include "math.h" // for pow() in readAltitude()

#include "bmpe.h"
#include "spi.h"

    /*=========================================================================
        I2C ADDRESS/BITS
        -----------------------------------------------------------------------*/
        #define BMP280_ADDRESS                (0x77)
    /*=========================================================================*/

    /*=========================================================================
        REGISTERS
        -----------------------------------------------------------------------*/
    // NOTE: strict subset of BME280 registers
        enum
        {
          BMP280_REGISTER_DIG_T1              = 0x88,
          BMP280_REGISTER_DIG_T2              = 0x8A,
          BMP280_REGISTER_DIG_T3              = 0x8C,

          BMP280_REGISTER_DIG_P1              = 0x8E,
          BMP280_REGISTER_DIG_P2              = 0x90,
          BMP280_REGISTER_DIG_P3              = 0x92,
          BMP280_REGISTER_DIG_P4              = 0x94,
          BMP280_REGISTER_DIG_P5              = 0x96,
          BMP280_REGISTER_DIG_P6              = 0x98,
          BMP280_REGISTER_DIG_P7              = 0x9A,
          BMP280_REGISTER_DIG_P8              = 0x9C,
          BMP280_REGISTER_DIG_P9              = 0x9E,

          BMP280_REGISTER_CHIPID             = 0xD0,
          BMP280_REGISTER_VERSION            = 0xD1,
          BMP280_REGISTER_SOFTRESET          = 0xE0,

          BMP280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

          BMP280_REGISTER_CONTROL            = 0xF4,
          BMP280_REGISTER_CONFIG             = 0xF5,
          BMP280_REGISTER_PRESSUREDATA       = 0xF7,
          BMP280_REGISTER_TEMPDATA           = 0xFA,
        };

    /*=========================================================================*/

        /*=========================================================================
            CALIBRATION DATA
            -----------------------------------------------------------------------*/
            typedef struct s_bmpe280_calib_data
            {
              uint16_t dig_T1;
              int16_t  dig_T2;
              int16_t  dig_T3;

              uint16_t dig_P1;
              int16_t  dig_P2;
              int16_t  dig_P3;
              int16_t  dig_P4;
              int16_t  dig_P5;
              int16_t  dig_P6;
              int16_t  dig_P7;
              int16_t  dig_P8;
              int16_t  dig_P9;

              uint8_t  dig_H1;
              int16_t  dig_H2;
              uint8_t  dig_H3;
              int16_t  dig_H4;
              int16_t  dig_H5;
              int8_t   dig_H6;
            } bmp280_calib_data;
        /*=========================================================================*/

            /*=========================================================================
                I2C ADDRESS/BITS
                -----------------------------------------------------------------------*/
                #define BME280_ADDRESS                (0x77)
            /*=========================================================================*/

            /*=========================================================================
                REGISTERS
                -----------------------------------------------------------------------*/
                enum
                {
                  BME280_REGISTER_DIG_T1              = BMP280_REGISTER_DIG_T1 ,
                  BME280_REGISTER_DIG_T2              = BMP280_REGISTER_DIG_T2 ,
                  BME280_REGISTER_DIG_T3              = BMP280_REGISTER_DIG_T3 ,

                  BME280_REGISTER_DIG_P1              = BMP280_REGISTER_DIG_P1 ,
                  BME280_REGISTER_DIG_P2              = BMP280_REGISTER_DIG_P2 ,
                  BME280_REGISTER_DIG_P3              = BMP280_REGISTER_DIG_P3 ,
                  BME280_REGISTER_DIG_P4              = BMP280_REGISTER_DIG_P4 ,
                  BME280_REGISTER_DIG_P5              = BMP280_REGISTER_DIG_P5 ,
                  BME280_REGISTER_DIG_P6              = BMP280_REGISTER_DIG_P6 ,
                  BME280_REGISTER_DIG_P7              = BMP280_REGISTER_DIG_P7 ,
                  BME280_REGISTER_DIG_P8              = BMP280_REGISTER_DIG_P8 ,
                  BME280_REGISTER_DIG_P9              = BMP280_REGISTER_DIG_P9 ,

                  BME280_REGISTER_DIG_H1              = 0xA1,
                  BME280_REGISTER_DIG_H2              = 0xE1,
                  BME280_REGISTER_DIG_H3              = 0xE3,
                  BME280_REGISTER_DIG_H4              = 0xE4,
                  BME280_REGISTER_DIG_H5              = 0xE5,
                  BME280_REGISTER_DIG_H6              = 0xE7,

                  BME280_REGISTER_CHIPID             = BMP280_REGISTER_CHIPID   ,
                  BME280_REGISTER_VERSION            = BMP280_REGISTER_VERSION  ,
                  BME280_REGISTER_SOFTRESET          = BMP280_REGISTER_SOFTRESET,

                  BME280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

                  BME280_REGISTER_CONTROLHUMID       = 0xF2,
                  BME280_REGISTER_CONTROL            = BMP280_REGISTER_CONTROL     ,
                  BME280_REGISTER_CONFIG             = BMP280_REGISTER_CONFIG      ,
                  BME280_REGISTER_PRESSUREDATA       = BMP280_REGISTER_PRESSUREDATA,
                  BME280_REGISTER_TEMPDATA           = BMP280_REGISTER_TEMPDATA    ,
                  BME280_REGISTER_HUMIDDATA          = 0xFD,
                };

            /*=========================================================================*/

/*=========================================================================
        CALIBRATION DATA
        -----------------------------------------------------------------------*/
    // NOTE: same calibration type as BME280 allows for normalized coefficients and no conditional compilation
        typedef bmp280_calib_data bme280_calib_data;
    /*=========================================================================*/

	//  private (low-level interface to SPI module - original also used I2C)
	typedef uint8_t byte;
	#define LOW (0)
	#define HIGH (1)
	#define INPUT (0)
	#define OUTPUT (1)

	//  private: module local variables and functions
    void readCoefficients(void);

    void		digitalWrite(byte x, byte value);
    byte		digitalRead(byte x);
    void		pinMode(byte x, byte value);
    byte		spixfer(byte x);

    void      write8(byte reg, byte value);
    uint8_t   read8(byte reg);
    uint16_t  read16(byte reg);
    uint32_t  read24(byte reg);
    int16_t   readS16(byte reg);
    uint16_t  read16_LE(byte reg); // little endian
    int16_t   readS16_LE(byte reg); // little endian

    uint8_t   _i2caddr;
    uint8_t   _chipID;
    int32_t   _sensorID;
    int32_t    t_fine; // humidity and pressure require temperature first

    int forcedMode = 1; // this is simplest to program, but cannot use on-chip IIR filter

    int8_t _cs, _mosi, _miso, _sck;

    bme280_calib_data _bme280_calib;

    int isBME280() { return _chipID == 0x60; }
    int isBMP280() { return _chipID == 0x58; }
    int bmpe_hasHumidity() { return isBME280(); }

    int bmpe_init(void) {
      //_i2caddr = a;
    	_cs = 16; // code for P0.16 the master SSEL pin
    	_mosi = 18;
    	_miso = 17;
    	_sck = 15; // indicates SOFTWARE SPI mode (SLOW, TESTING ONLY!)
    	//_sck = -1; // HARDWARE SPI mode

    	spi_init(); // this is required FIRST for LPC-1769 but NOT Arduino, and both SW and HW modes
    	// from the Arduino docs for SPI.begin():
    	// "Initializes the SPI bus by setting SCK, MOSI, and SS to outputs, pulling SCK and MOSI low, and SS high."
    	// NOTE that is pulls SSEL high and the other outputs low.
    	// On the 1769, it will just set up the pins for proper functional operation and pull-up resistor mode (unused in Arduino)

    	// set up the SSEL pin for either mode: level before direction
		digitalWrite(_cs, HIGH);
		pinMode(_cs, OUTPUT);

		if (_sck != -1) {
			// NOTE: Arduino code would call SPI.begin() here, after setting SSEL out high = maybe this was added to SPI later on?
			// software SPI
			digitalWrite(_sck, LOW); // not in current Arduino code, function of SPI.begin()
			digitalWrite(_mosi, LOW); // not in current Arduino code, function of SPI.begin()
			pinMode(_sck, OUTPUT);
			pinMode(_mosi, OUTPUT);
			pinMode(_miso, INPUT);
		}

      _chipID = read8(BME280_REGISTER_CHIPID);
      _chipID = 0x58; // DEBUG OVERRIDE FOR TESTING ONLY!
      if (_chipID != 0x60 && _chipID != 0x58)
        return BMPE_NONE;

      readCoefficients();

      if (forcedMode) {
		  // BMP: Forced mode xfers (one at a time, no filter)
		  write8(BMP280_REGISTER_CONTROL, 0x3F);
      } else {
		  //Set before CONTROL_meas (DS 5.4.3)
		  write8(BME280_REGISTER_CONTROLHUMID, 0x05); //16x oversampling

		  write8(BME280_REGISTER_CONTROL, 0xB7); // 16x ovesampling, normal mode
      }

      return bmpe_hasHumidity()? BMPE_BMETYPE: BMPE_BMPTYPE;
    }

// ORIGINAL ADAFRUIT LIBRARIES ARE ON GITHUB AT:
    // These libraries were designed to interface to functions that could interface to I2C, SPI HW, and a software emulation of SPI.
    // There is no need for me to keep all that, so I pared it down to this.
    // UPDATE: Added back in the emulation SPI mode for testing
    uint8_t spixfer(uint8_t x) {
    	if (_sck == -1)
    		return spi_transfer(x);

		// software spi
		//Serial.println("Software SPI");
		uint8_t reply = 0;
		for (int i=7; i>=0; i--) {
			reply <<= 1;
			digitalWrite(_sck, LOW);
			digitalWrite(_mosi, x & (1<<i));
			digitalWrite(_sck, HIGH);
			if (digitalRead(_miso))
			  reply |= 1;
		}
		return reply;
    }

    void digitalWrite(uint8_t pin, uint8_t x) {
    	spi_pinWrite(pin, x);
    }

    uint8_t digitalRead(uint8_t pin) {
    	return spi_pinRead(pin);
    }

    void pinMode(uint8_t pin, uint8_t x) {
    	spi_pinMode(pin, x);
    }

    /**************************************************************************/
    /*!
        @brief  Writes an 8 bit value over SPI
    */
    /**************************************************************************/
    void write8(byte reg, byte value)
    {
		if (_sck == -1)
	        spi_beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
		digitalWrite(_cs, LOW);
		spixfer(reg & ~0x80); // write, bit 7 low
		spixfer(value);
		digitalWrite(_cs, HIGH);
		if (_sck == -1)
			spi_endTransaction();              // release the SPI bus
    }

    /**************************************************************************/
    /*!
        @brief  Reads an 8 bit value over SPI
    */
    /**************************************************************************/
    uint8_t read8(byte reg)
    {
      uint8_t value;

      if (_sck == -1)
        spi_beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
      digitalWrite(_cs, LOW);
      spixfer(reg | 0x80); // read, bit 7 high
      value = spixfer(0);
      digitalWrite(_cs, HIGH);
      if (_sck == -1)
    	  spi_endTransaction();              // release the SPI bus
      return value;
    }

    /**************************************************************************/
    /*!
        @brief  Reads a 16 bit value over SPI
    */
    /**************************************************************************/
    uint16_t read16(byte reg)
    {
      uint16_t value;

      if (_sck == -1)
    	  spi_beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
      digitalWrite(_cs, LOW);
      spixfer(reg | 0x80); // read, bit 7 high
      value = (spixfer(0) << 8) | spixfer(0);
      digitalWrite(_cs, HIGH);
      if (_sck == -1)
    	  spi_endTransaction();              // release the SPI bus

      return value;
    }

    uint16_t read16_LE(byte reg) {
      uint16_t temp = read16(reg);
      return (temp >> 8) | (temp << 8);

    }

    /**************************************************************************/
    /*!
        @brief  Reads a signed 16 bit value over SPI
    */
    /**************************************************************************/
    int16_t readS16(byte reg)
    {
      return (int16_t)read16(reg);

    }

    int16_t readS16_LE(byte reg)
    {
      return (int16_t)read16_LE(reg);

    }


    /**************************************************************************/
    /*!
        @brief  Reads a 24 bit value over SPI
    */
    /**************************************************************************/

    uint32_t read24(byte reg)
    {
      uint32_t value;

      if (_sck == -1)
    	 spi_beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
      digitalWrite(_cs, LOW);
      spixfer(reg | 0x80); // read, bit 7 high

      value = spixfer(0);
      value <<= 8;
      value |= spixfer(0);
      value <<= 8;
      value |= spixfer(0);

      digitalWrite(_cs, HIGH);
      if (_sck == -1)
    	 spi_endTransaction();              // release the SPI bus

      return value;
    }


    /**************************************************************************/
    /*!
        @brief  Reads the factory-set coefficients
    */
    /**************************************************************************/
    void readCoefficients(void)
    {
        _bme280_calib.dig_T1 = read16_LE(BME280_REGISTER_DIG_T1);
        _bme280_calib.dig_T2 = readS16_LE(BME280_REGISTER_DIG_T2);
        _bme280_calib.dig_T3 = readS16_LE(BME280_REGISTER_DIG_T3);

        _bme280_calib.dig_P1 = read16_LE(BME280_REGISTER_DIG_P1);
        _bme280_calib.dig_P2 = readS16_LE(BME280_REGISTER_DIG_P2);
        _bme280_calib.dig_P3 = readS16_LE(BME280_REGISTER_DIG_P3);
        _bme280_calib.dig_P4 = readS16_LE(BME280_REGISTER_DIG_P4);
        _bme280_calib.dig_P5 = readS16_LE(BME280_REGISTER_DIG_P5);
        _bme280_calib.dig_P6 = readS16_LE(BME280_REGISTER_DIG_P6);
        _bme280_calib.dig_P7 = readS16_LE(BME280_REGISTER_DIG_P7);
        _bme280_calib.dig_P8 = readS16_LE(BME280_REGISTER_DIG_P8);
        _bme280_calib.dig_P9 = readS16_LE(BME280_REGISTER_DIG_P9);

        if (bmpe_hasHumidity()) {
			_bme280_calib.dig_H1 = read8(BME280_REGISTER_DIG_H1);
			_bme280_calib.dig_H2 = readS16_LE(BME280_REGISTER_DIG_H2);
			_bme280_calib.dig_H3 = read8(BME280_REGISTER_DIG_H3);
			_bme280_calib.dig_H4 = (read8(BME280_REGISTER_DIG_H4) << 4) | (read8(BME280_REGISTER_DIG_H4+1) & 0xF);
			_bme280_calib.dig_H5 = (read8(BME280_REGISTER_DIG_H5+1) << 4) | (read8(BME280_REGISTER_DIG_H5) >> 4);
			_bme280_calib.dig_H6 = (int8_t)read8(BME280_REGISTER_DIG_H6);
        }
    }

    /**************************************************************************/
    /*!
    */
    /**************************************************************************/
    float bmpe_readTemperature(void)
    {
      int32_t var1, var2;

      int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);
      adc_T >>= 4;

      var1  = ((((adc_T>>3) - ((int32_t)_bme280_calib.dig_T1 <<1))) *
    	   ((int32_t)_bme280_calib.dig_T2)) >> 11;

      var2  = (((((adc_T>>4) - ((int32_t)_bme280_calib.dig_T1)) *
    	     ((adc_T>>4) - ((int32_t)_bme280_calib.dig_T1))) >> 12) *
    	   ((int32_t)_bme280_calib.dig_T3)) >> 14;

      t_fine = var1 + var2;

      float T  = (t_fine * 5 + 128) >> 8;
      return T/100;
    }

    /**************************************************************************/
    /*!
    */
    /**************************************************************************/
    float bmpe_readPressure(void) {
      int64_t var1, var2, p;

      bmpe_readTemperature(); // must be done first to get t_fine

      int32_t adc_P = read24(BME280_REGISTER_PRESSUREDATA);
      adc_P >>= 4;

      var1 = ((int64_t)t_fine) - 128000;
      var2 = var1 * var1 * (int64_t)_bme280_calib.dig_P6;
      var2 = var2 + ((var1*(int64_t)_bme280_calib.dig_P5)<<17);
      var2 = var2 + (((int64_t)_bme280_calib.dig_P4)<<35);
      var1 = ((var1 * var1 * (int64_t)_bme280_calib.dig_P3)>>8) +
        ((var1 * (int64_t)_bme280_calib.dig_P2)<<12);
      var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bme280_calib.dig_P1)>>33;

      if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
      }
      p = 1048576 - adc_P;
      p = (((p<<31) - var2)*3125) / var1;
      var1 = (((int64_t)_bme280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
      var2 = (((int64_t)_bme280_calib.dig_P8) * p) >> 19;

      p = ((p + var1 + var2) >> 8) + (((int64_t)_bme280_calib.dig_P7)<<4);
      return (float)p/256;
    }


    /**************************************************************************/
    /*!
    */
    /**************************************************************************/
    float bmpe_readHumidity(void) {

        if (!bmpe_hasHumidity()) {
        	return 0.0;
        }

      bmpe_readTemperature(); // must be done first to get t_fine

      int32_t adc_H = read16(BME280_REGISTER_HUMIDDATA);

      int32_t v_x1_u32r;

      v_x1_u32r = (t_fine - ((int32_t)76800));

      v_x1_u32r = (((((adc_H << 14) - (((int32_t)_bme280_calib.dig_H4) << 20) -
    		  (((int32_t)_bme280_calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
    	       (((((((v_x1_u32r * ((int32_t)_bme280_calib.dig_H6)) >> 10) *
    		    (((v_x1_u32r * ((int32_t)_bme280_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
    		  ((int32_t)2097152)) * ((int32_t)_bme280_calib.dig_H2) + 8192) >> 14));

      v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
    			     ((int32_t)_bme280_calib.dig_H1)) >> 4));

      v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
      v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
      float h = (v_x1_u32r>>12);
      return  h / 1024.0;
    }

    /**************************************************************************/
    /*!
        Calculates the altitude (in meters) from the current and specified
        reference atmospheric pressures (in hPa).
    */
    /**************************************************************************/
    // The seaLevel pressure used here is the standard for one atmosphere, in hPa.
#define STD_ATMOSPHERE (1013.25F / 100.0F)
    float seaLevel = STD_ATMOSPHERE; // TODO: create a method for calibrating this somehow

    void bmpe_setReferencePressure() {
    	seaLevel = bmpe_readPressure() / 100.0F;
    }

    float bmpe_readAltitude()
    {
      // Equation taken from BMP180 datasheet (page 16):
      //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

      // Note that using the equation from wikipedia can give bad results
      // at high altitude.  See this thread for more information:
      //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

      float atmospheric = bmpe_readPressure() / 100.0F;
      return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
    }
