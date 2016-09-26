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
 *  Added a method to set the sea-level (reference) pressure to allow relative altitude calculations.
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

#include "stdio.h" // for DBGOUT()
#include "lpc_types.h" // for TRUE and FALSE
//#define DEBUGOUT // uncomment this for debugging statements
//#define DOUBLE_PRECISION // uncomment this for full-precision reference compensation (direct from data sheet)

#ifdef DEBUGOUT
#define DBGOUT printf
#else
#define DBGOUT
#endif

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

          BMP280_REGISTER_STATUS             = 0xF3,

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

		          BME280_REGISTER_STATUS             = BMP280_REGISTER_STATUS,

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
    void showTimingParameters(byte crConfig, byte crControl, byte crControlH, int isHumidOK);

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

    enum {
    	FILTERED=-1, // samples are created automatically at Tstandby approx.rate, filtering is applied
		NORMAL, // samples are created automatically as above, no filtering (Arduino compatible mode)
		FORCED,  // must request every sample (not really supported)
    };
    int operatingMode = FILTERED;
    byte regValueConfig=0x00, regValueControl=0x00, regValueControlH=0x00;

    int8_t _cs, _mosi, _miso, _sck;

    bme280_calib_data _bme280_calib;

    int isBME280() { return _chipID == 0x60; }
    int isBMP280() { return _chipID == 0x58; }
    int bmpe_hasHumidity() { return isBME280(); }

    int bmpe_init(void) {
      //_i2caddr = a;
    	_cs = 16; // code for P0.16 the master SSEL pin
    	//_cs = 77; // code for P2.13 an alternate SSEL pin (2*32+13 = 77)
    	_mosi = 18;
    	_miso = 17;
    	_sck = 15; // indicates SOFTWARE SPI mode (BIT BANGING, SLOW, TESTING ONLY!)
    	_sck = -1; // HARDWARE SPI mode

    	spi_init(_cs); // this is required FIRST for LPC-1769 but NOT Arduino, and both SW and HW modes
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

      // need to set SLEEP mode to read registers and coefficients
	  write8(BMP280_REGISTER_CONTROL, 0x00); // reset value is 000 000 00b (skip Temp and Press (set to MSBIT), SLEEP mode)
	  write8(BMP280_REGISTER_CONFIG, 0x00); // reset value is 000 000 00b (no filter, fast Tstandby, no 3-wire SPI)

      _chipID = read8(BME280_REGISTER_CHIPID);
      if (_chipID != 0x60 && _chipID != 0x58) // NOTE: chip pre-productions samples of BMP also returned 0x56 or 0x57
        return BMPE_NONE;

      readCoefficients();

      // set config and control registers (incl.control for humidity if needed)
      if (operatingMode == NORMAL) {
    	  // OLD WAY OF "FORCED MODE"; I was actually using oversampled NORMAL mode w/o the filtering
    	  if (bmpe_hasHumidity())
    		  regValueControlH = (1 << 0); // humid 1x sampling (reset value is SKIP/0, sets 20-bit data to MSBIT (0x80000)
    	  regValueConfig = (0x0 << 5) + (0x0 << 2) + (0x0 << 0); //0x00// reset value is 000 000 00b (fast Tstandby, no filter, no 3-wire SPI)
		  regValueControl = (0x1 << 5) + (0x7 << 2) + (0x3 << 0); //0x3F// 001 111 11b Temp.1x, Press.16x, NORMAL mode (oops!)
      } else if (operatingMode == FORCED) {
		  // BMP: Forced mode xfers (one at a time, no filter)
		  //Set before CONTROL_meas (DS 5.4.3)
    	  if (bmpe_hasHumidity())
    		  regValueControlH = (1 << 0); // humid 1x sampling (reset value is SKIP/0, sets 20-bit data to MSBIT (0x80000)
    	  regValueConfig = (0x0 << 5) + (0x0 << 2) + (0x0 << 0); //0x00// reset value is 000 000 00b (no filter, fast Tstandby, no 3-wire SPI)
    	  regValueControl = (0x1 << 5) + (0x1 << 2) + (0x0 << 0); //0x25// 001 001 01b Temp.1x, Press.1x, SLEEP mode
		  // NOTE: actual use of this requires resending the forced mode control byte EVERY TIME A SAMPLE IS DESIRED (reverts to sleep mode after completion)
		  // This would also imply reading the status register to wait for completion of that sample.
      } else {
    	  /*
    	   * NOTE: For usage in delta-height feature, we want lowest noise and highest accuracy. No need to oversample humidity, tho, it doesn't get noisy.
    	   * According to the datasheet Sec 3.5.3 (p.18 and Table 9), we want the following settings in CONTROL and CONFIG registers
    	   * 	CONFIG.7:5 (3b)  = 000 (0 << 5)		Tstandby = 0.5ms (fastest) [0=0.5msec, 1=62.5, 2=125, 3=250, 4=500, 5=1000, 6=2000(P)/10(E), 7=4000(P)/20(E)]
    	   * 	CONFIG.4:2 (3b)  = 100 (4 << 2)		Filter ON using 16 coeffs [0=OFF, 1=2, 2=4, 3=8, 4=16, 5+=reserved]
    	   * 	CONFIG.1:0 (2b)  =  00 (0 << 0)		Use 3-wire SPI OFF [1=ON, 2,3 are reserved - DO NOT USE]
    	   *
    	   * 	CTROL_H.2:0 (3b) = 001 (1 << 0)		Humidity 1x oversampling [ same meaning as Temperature ]
    	   *
    	   * 	CONTROL.7:5 (3b) = 010 (2 << 5)		Temperature 2x oversampling [0=SKIP, 1=1x, 2=2x, 3=4x, 4=8x, 5=16x, 6+=reserved]
    	   * 	CONTROL.4:2 (3b) = 101 (5 << 2)		Pressure 16x oversampling [ same meaning as Temperature ]
    	   * 	CONTROL.1:0 (2b) =  11 (3 << 0)		Normal mode [0 is sleep, 1/2 is forced, 3 is normal]
    	   *
    	   * NOTE: This mode will also require Burst Mode Reads to avoid the problem solved by the Shadow Registers (see DS 4.1 p.21)
    	   */
		  //Set before CONTROL_meas (DS 5.4.3)
    	  if (bmpe_hasHumidity())
        	  regValueControlH = (1 << 0); // humid 1x sampling (reset value is SKIP/0, sets 16-bit data to MSBIT (0x8000)
		  regValueConfig = (0x0 << 5) + (0x2 << 2) + (0x0 << 0); //0x08//000(Tsb) 010(F) 00(3W)// fastest sampling/lowest Tstandby, 4x.filtering, no 3-wire SPI
		  regValueControl = (0x2 << 5) + (0x5 << 2) + (0x3 << 0); //0x57//010(T) 101(P) 11(M)// 16x oversampling Press, 2x ovs.on Temp, normal mode
		  // NOTE: in NORMAL mode, register writes are disabled until SLEEP mode is sent
      }
      // send the control register values selected
	  if (bmpe_hasHumidity())
		  write8(BME280_REGISTER_CONTROLHUMID, regValueControlH);
	  write8(BME280_REGISTER_CONFIG, regValueConfig);
	  write8(BME280_REGISTER_CONTROL, regValueControl);
	  // calculate and show relevant timing parameters on debug console (OPT)
	  showTimingParameters(regValueConfig, regValueControl, regValueControlH, bmpe_hasHumidity());

      return bmpe_hasHumidity()? BMPE_BMETYPE: BMPE_BMPTYPE;
    }

    void bmpe_measureForcedMode() {
    	// send the control byte with mode bits 1:0 set to b01 (FORCED)
  	  write8(BME280_REGISTER_CONTROL, regValueControl | 0x01);
    	// wait for status byte to return OK
  	  byte mask = (1 << 3) | (1 << 0); // two status bits, for NVM copying and for conversion, both must be 0 in ready state
  	  // NOTE: hopefully the state transitions of the two bits are 8 >> 1 >> 0 - if there is an intermediate 0, we will have a false done signal and need two loops
  	  while ((read8(BME280_REGISTER_STATUS) & mask) != 0x00)
  		  ;
    	// DEBUG: check humidity data - why is it always 0 in burst mode?
  	  byte humidL = read8(BME280_REGISTER_HUMIDDATA);
  	  byte humidH = read8(BME280_REGISTER_HUMIDDATA + 1);
  	  int humid = (humidH << 8) + humidL;
      DBGOUT("bmpeRdH8(r%02x):%02x %02x = %04x(%d)\n", BME280_REGISTER_HUMIDDATA, (int)(humidH), (int)(humidL), humid, humid);
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
      DBGOUT("bmpeRd8(r%02x):%02x\n", reg, (int)(value & 0xFF));
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

      DBGOUT("bmpeRd16(r%02x):%02x/%02x\n", reg, (int)(value & 0xFF00)>>8, (int)(value & 0xFF));
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

      DBGOUT("bmpeRd24(r%02x):%02x/%02x/%02x\n", reg, (int)(value & 0xFF0000)>>16, (int)(value & 0xFF00)>>8, (int)(value & 0xFF));
      return value;
    }


    /**************************************************************************/
    /*!
        @brief  Reads all uncompensated data values over SPI in burst mode (one read command)
        ORDER: P(3r) - T(3r) - H(2r)
        Allowed values for length:
        3 = equivalent to read24(), assembles 1st value from 3 regs (separate P, T, or H)
        5 = assembles and returns 1st two values (3 regs. + 2 regs.) (T+H)
        6 = assembles and returns 1st two values (3 regs.each) (P+T)
        8 = assembles and returns all 3 values (3+3+2 regs) (P+T+H)
    */
    /**************************************************************************/

    void readBurst(byte reg, int length, int32_t* psValueA, int32_t* psValueB, int32_t* psValueC)
    {
      uint32_t value1, value2, value3;

      if (_sck == -1)
    	 spi_beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
      digitalWrite(_cs, LOW);
      spixfer(reg | 0x80); // read, bit 7 high

      // NOTE: in burst mode, after the single read command, the register numbers auto-increment
      // this works in read16()/read24() as well, but only for 2/3 registers

      if (length >= 3) {
    	  // first 3 bytes are 1st value
          value1 = spixfer(0);
          value1 <<= 8;
          value1 |= spixfer(0);
          value1 <<= 8;
          value1 |= spixfer(0);
          if (psValueA != NULL)
        	  *psValueA = value1;
      }

      if (length >= 6) {
    	  // next 3 bytes are 2nd value
          value2 = spixfer(0);
          value2 <<= 8;
          value2 |= spixfer(0);
	      if (length > 5) {
			  value2 <<= 8;
			  value2 |= spixfer(0);
	      }
          if (psValueB != NULL)
        	  *psValueB = value2;
      }

      if (length > 6) {
    	  // last 2 or 3 bytes are 3rd value
		  value3 = spixfer(0);
		  value3 <<= 8;
		  value3 |= spixfer(0);
	      if (length > 8) {
			  value3 <<= 8;
			  value3 |= spixfer(0);
	      }
	      if (psValueC != NULL)
	    	  *psValueC = value3;
      }

      digitalWrite(_cs, HIGH);				// this stops burst mode reading
      if (_sck == -1)
    	 spi_endTransaction();              // release the SPI bus

      DBGOUT("bmpeRdB(r%02x):%02x/%02x/%02x\n", reg, (int)(value1 & 0xFF0000)>>16, (int)(value1 & 0xFF00)>>8, (int)(value1 & 0xFF));
      DBGOUT("bmpeRdB(r%02x):%02x/%02x/%02x\n", reg+3, (int)(value2 & 0xFF0000)>>16, (int)(value2 & 0xFF00)>>8, (int)(value2 & 0xFF));
      if (length > 8)
    	  DBGOUT("bmpeRdB(r%02x):%02x/%02x/%02x\n", reg+6, (int)(value3 & 0xFF0000)>>16, (int)(value3 & 0xFF00)>>8, (int)(value3 & 0xFF));
      else if (length > 6)
    	  DBGOUT("bmpeRdB(r%02x):%02x/%02x\n", reg+6, (int)(value3 & 0xFF00)>>8, (int)(value3 & 0xFF));
      return;
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
    /* returns value in degrees C (0.01 deg.C precision)
    */
    /**************************************************************************/
#ifndef DOUBLE_PRECISION
    static float compensateTemperature(int32_t adc_T)
    {
      int32_t var1, var2;

      adc_T >>= 4;

      var1  = ((((adc_T>>3) - ((int32_t)_bme280_calib.dig_T1 <<1))) *
    	   ((int32_t)_bme280_calib.dig_T2)) >> 11;

      var2  = (((((adc_T>>4) - ((int32_t)_bme280_calib.dig_T1)) *
    	     ((adc_T>>4) - ((int32_t)_bme280_calib.dig_T1))) >> 12) *
    	   ((int32_t)_bme280_calib.dig_T3)) >> 14;

      t_fine = var1 + var2; // save globally for pressure and humidity compensation

      float T  = (t_fine * 5 + 128) >> 8;
      return T/100;
    }
#else
    /*
     * Double-precision Reference version from Bosch data sheet Appendix A.
     */
	typedef uint32_t BME280_S32_t;
	// Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
	// t_fineX carries fine temperature as global value
	BME280_S32_t t_fineX;
	static double BME280_compensate_T_double(BME280_S32_t adc_T)
	{
		BME280_S32_t dig_T1 = _bme280_calib.dig_T1;
		BME280_S32_t dig_T2 = _bme280_calib.dig_T2;
		BME280_S32_t dig_T3 = _bme280_calib.dig_T3;
		double var1, var2, T;
		var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
		var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) *
				(((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
		t_fineX = (BME280_S32_t)(var1 + var2);
		T = (var1 + var2) / 5120.0;
		return T;
	}
	/* END D.P.REF.COMPENSATION TEMPERATURE IMPLEMENTATION */
#endif

    float bmpe_readTemperature(void)
    {
      int32_t var1, var2;

      int32_t adc_T;
      readBurst(BME280_REGISTER_TEMPDATA, 3, &adc_T, NULL, NULL);
#ifndef DOUBLE_PRECISION
      return compensateTemperature(adc_T);
#else
      return (float)BME280_compensate_T_double(adc_T);
#endif
    }

    /**************************************************************************/
    /* returns value in Pa (pascals); must divide by 100.0 to get hPa (more common)
    */
    /**************************************************************************/
#ifndef DOUBLE_PRECISION
    static float compensatePressure(int32_t adc_P) {
      int64_t var1, var2, p;

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
#else
    /*
     * Double-precision Reference version from Bosch data sheet Appendix A.
     */
    // Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
    static double BME280_compensate_P_double(BME280_S32_t adc_P)
    {
    	BME280_S32_t dig_P1 = _bme280_calib.dig_P1;
    	BME280_S32_t dig_P2 = _bme280_calib.dig_P2;
    	BME280_S32_t dig_P3 = _bme280_calib.dig_P3;
    	BME280_S32_t dig_P4 = _bme280_calib.dig_P4;
    	BME280_S32_t dig_P5 = _bme280_calib.dig_P5;
    	BME280_S32_t dig_P6 = _bme280_calib.dig_P6;
    	BME280_S32_t dig_P7 = _bme280_calib.dig_P7;
    	BME280_S32_t dig_P8 = _bme280_calib.dig_P8;
    	BME280_S32_t dig_P9 = _bme280_calib.dig_P9;
    	double var1, var2, p;
    	var1 = ((double)t_fineX/2.0) - 64000.0;
    	var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
    	var2 = var2 + var1 * ((double)dig_P5) * 2.0;
    	var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
    	var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
    	var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
    	if (var1 == 0.0)
    	{
    		return 0; // avoid exception caused by division by zero
    	}
    	p = 1048576.0 - (double)adc_P;
    	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    	var1 = ((double)dig_P9) * p * p / 2147483648.0;
    	var2 = p * ((double)dig_P8) / 32768.0;
    	p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
    	return p;
    }
	/* END D.P.REF.COMPENSATION PRESSURE IMPLEMENTATION */
#endif

    float bmpe_readPressure(void) {
      int32_t adc_T, adc_P;

      readBurst(BME280_REGISTER_PRESSUREDATA, 6, &adc_P, &adc_T, NULL);

#ifndef DOUBLE_PRECISION
      compensateTemperature(adc_T); // must be done first to get t_fine

      return compensatePressure(adc_P);
#else
      BME280_compensate_T_double(adc_T); // must be done first to get t_fine

      return (float)BME280_compensate_P_double(adc_P);
#endif
    }


    /**************************************************************************/
    /* HUMIDITY - read singly as 2-byte 'burst'
    */
    /**************************************************************************/
#ifndef DOUBLE_PRECISION
    static float compensateHumidity(int32_t adc_H) {

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
#else
    /*
     * Double-precision Reference version from Bosch data sheet Appendix A.
     */
    // Returns humidity in %rH as as double. Output value of “46.332” represents 46.332 %rH
    static double BME280_compensate_H_double(BME280_S32_t adc_H)
    {
    	BME280_S32_t dig_H1 = _bme280_calib.dig_H1;
    	BME280_S32_t dig_H2 = _bme280_calib.dig_H2;
    	BME280_S32_t dig_H3 = _bme280_calib.dig_H3;
    	BME280_S32_t dig_H4 = _bme280_calib.dig_H4;
    	BME280_S32_t dig_H5 = _bme280_calib.dig_H5;
    	BME280_S32_t dig_H6 = _bme280_calib.dig_H6;
    	double var_H;
    	var_H = (((double)t_fineX) - 76800.0);
    	var_H = (adc_H - (((double)dig_H4) * 64.0 + ((double)dig_H5) / 16384.0 * var_H)) *
			(((double)dig_H2) / 65536.0 * (1.0 + ((double)dig_H6) / 67108864.0 * var_H *
			(1.0 + ((double)dig_H3) / 67108864.0 * var_H)));
    	var_H = var_H * (1.0 - ((double)dig_H1) * var_H / 524288.0);
    	if (var_H > 100.0)
    		var_H = 100.0;
    	else if (var_H < 0.0)
    		var_H = 0.0;
    	return var_H;
    }
	/* END D.P.REF.COMPENSATION HUMIDITY IMPLEMENTATION */
#endif

    float bmpe_readHumidity(void) {

        if (!bmpe_hasHumidity()) {
        	return 0.0;
        }

        int32_t adc_T, adc_H;

        readBurst(BME280_REGISTER_TEMPDATA, 5, &adc_T, &adc_H, NULL);

#ifndef DOUBLE_PRECISION
        compensateTemperature(adc_T); // must be done first to get t_fine

        return compensateHumidity(adc_H);
#else
        BME280_compensate_T_double(adc_T); // must be done first to get t_fine

        return (float)BME280_compensate_H_double(adc_H);
#endif
    }

    /**************************************************************************/
    /*!
        Calculates the altitude (in meters) from the current and specified
        reference atmospheric pressures (in Pa).
    */
    /**************************************************************************/
    // The seaLevel pressure used here is the standard for one atmosphere, in Pa.
#define STD_ATMOSPHERE (101325.0F)
    float seaLevel = STD_ATMOSPHERE;

    static float calcAltitude(float atmospheric, float seaLevel)
    {
      // Equation taken from BMP180 datasheet (page 16):
      //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

      // Note that using the equation from wikipedia can give bad results
      // at high altitude.  See this thread for more information:
      //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064
    	double atm = atmospheric;
    	double sea = seaLevel;
    	double ratio = atm / sea;
    	double temp = pow(ratio, 0.1903);
    	double diff = 1.0 - temp;
    	double value = 44330.0 * diff;

      return (float)value;
    }

    void bmpe_setReferencePressure() {
    	// override the standard sea level pressure with the current reading in Pa
    	bmpe_readAllSensors(NULL, &seaLevel, NULL, NULL);
    	double temp = calcAltitude(seaLevel, seaLevel);
    	DBGOUT("Ref.Pressure = %1.5f Pa (%1.5f m)", seaLevel, temp);
    }

    float bmpe_readAltitude()
    {
      float atmospheric = bmpe_readPressure();
      return calcAltitude(atmospheric, seaLevel);
    }

    /**************************************************************************/
    /* Read all samples in a data burst (best for NORMAL mode reading)
     *  See DS 4/4.1 on Shadow registers and data readout.
     *  UPDATE: ORDER OF REGISTERS IS P, THEN T, THEN H
    */
    /**************************************************************************/
    void bmpe_readAllSensors(float* pfTemp, float* pfPress, float *pfAlt, float* pfHumid) {
    	int32_t adc_T, adc_P, adc_H;
    	if (operatingMode == FORCED)
    		bmpe_measureForcedMode();
    	int length = (bmpe_hasHumidity()? 8: 6);
    	readBurst(BME280_REGISTER_PRESSUREDATA, length, &adc_P, &adc_T, &adc_H);
    	float humid = 0.0;
#ifndef DOUBLE_PRECISION
    	float temp = compensateTemperature(adc_T); // must be done 1st to set t_fineX
    	float press = compensatePressure(adc_P);
    	if (length >= 8)
    		humid = compensateHumidity(adc_H);
#else
    	float temp = (float)BME280_compensate_T_double(adc_T); // must be done 1st to set t_fineX
    	float press = (float)BME280_compensate_P_double(adc_P);
    	if (length >= 8)
    		humid = (float)BME280_compensate_H_double(adc_H);
#endif
    	if (pfTemp)
    		*pfTemp = temp;
    	if (pfPress)
    		*pfPress = press;
    	if (pfAlt)
    		*pfAlt = calcAltitude(press, seaLevel);
		if (pfHumid)
			*pfHumid = humid;
    }

    /**************************************************************************/
    // calculate timing parameters as specified in Datasheet 9.1-9.4
    /**************************************************************************/
    float calcTmeasComponent(int osrs, float coeff1, float coeff2) {
    	float value = 0.0;
    	if (osrs != 0) {
    		value = osrs * coeff1 + coeff2;
    	}
    	return value;
    }

    float calcTmeas(int osrsT, int osrsP, int osrsH, float coeff1, float coeff2, float coeff3) {
    	float value = coeff3;
    	value += calcTmeasComponent(osrsT, coeff1, 0.0);
    	value += calcTmeasComponent(osrsP, coeff1, coeff2);
    	value += calcTmeasComponent(osrsH, coeff1, coeff2);
    	return value;
    }

    float calcTstandbyMsec(byte tsbCode, int humidOK) {
    	// Datasheet Table 27, p.28
    	float result = 0.5;
    	switch (tsbCode) {
    	case 1:
    		result = 62.5;
    		break;
    	case 2:
    		result = 125.0;
    		break;
    	case 3:
    		result = 250.0;
    		break;
    	case 4:
    		result = 500.0;
    		break;
    	case 5:
    		result = 1000.0;
    		break;
    	case 6:
    		result = (humidOK? 10.0: 2000.0);
    		break;
    	case 7:
    		result = (humidOK? 20.0: 4000.0);
    		break;
    	default:
    		break;
    	}
    	return result;
    }

    int getOversampling(byte osrsCode) {
    	// Datasheet Tables 20, 23, 24 on pp.26-27
    	int result = 0;
    	switch (osrsCode) {
    	case 1:
    		result = 1;
    		break;
    	case 2:
    		result = 2;
    		break;
    	case 3:
    		result = 4;
    		break;
    	case 4:
    		result = 8;
    		break;
    	case 5:
    		result = 16;
    		break;
    	default:
    		break;
    	}
    	return result;
    }

    int getFilterCoeffs(byte filterCode) {
    	// Datasheet Table 28, p.28
    	int result = 0;
    	switch (filterCode) {
    	case 1:
    		result = 2;
    		break;
    	case 2:
    		result = 4;
    		break;
    	case 3:
    		result = 8;
    		break;
    	case 4:
    		result = 16;
    		break;
    	default:
    		break;
    	}
    	return result;
    }

    int calcNSamplesTo75Pct(int filterCode) {
    	// Datasheet Table 6 p.16
    	int result = 1;
    	switch (filterCode) {
    	case 1: // 2 coeffs
    		result = 2;
    		break;
    	case 2: // 4 coeffs
    		result = 5;
    		break;
    	case 3: // 8 coeffs
    		result = 11;
    		break;
    	case 4: // 16 coeffs
    		result = 22;
    		break;
    	default:
    		break;
    	}
    	return result;
    }

    void showTimingParameters(byte crConfig, byte crControl, byte crControlH, int isHumidOK) {
		// extract the various fields
		/*
		* According to the datasheet Sec 3.5.3 (p.18 and Table 9), we want the following settings in CONTROL and CONFIG registers
		* 	CONFIG.7:5 (3b)  = 000 (0 << 5)		Tstandby = 0.5ms (fastest) [0=0.5msec, 1=62.5, 2=125, 3=250, 4=500, 5=1000, 6=2000(P)/10(E), 7=4000(P)/20(E)]
		* 	CONFIG.4:2 (3b)  = 100 (4 << 2)		Filter ON using 16 coeffs [0=OFF, 1=2, 2=4, 3=8, 4=16, 5+=reserved]
		* 	CONFIG.1:0 (2b)  =  00 (0 << 0)		Use 3-wire SPI OFF [1=ON, 2,3 are reserved - DO NOT USE]
		*
		* 	CTROL_H.2:0 (3b) = 001 (1 << 0)		Humidity 1x oversampling [ same meaning as Temperature ]
		*
		* 	CONTROL.7:5 (3b) = 010 (2 << 5)		Temperature 2x oversampling [0=SKIP, 1=1x, 2=2x, 3=4x, 4=8x, 5=16x, 6+=reserved]
		* 	CONTROL.4:2 (3b) = 101 (5 << 2)		Pressure 16x oversampling [ same meaning as Temperature ]
		* 	CONTROL.1:0 (2b) =  11 (3 << 0)		Normal mode [0 is sleep, 1/2 is forced, 3 is normal]
		*
		*/
    	const char* typeStr = (isHumidOK? "BME/TempPressAltHumid": "BMP/TempPressAlt");
    	byte t_standbyCode = (crConfig >> 5) & 0b111;
    	byte filter = (crConfig >> 2) & 0b111;
    	byte mode = (crControl >> 0) & 0b11;
    	byte osrs_tCode = (crControl >> 5) & 0b111;
    	byte osrs_pCode = (crControl >> 2) & 0b111;
    	byte osrs_hCode = (crControlH >> 0) & 0b111;
    	int forcedMode = (mode == 1 || mode == 2)? TRUE: FALSE;
    	int normalMode = (mode == 3)? TRUE: FALSE;
    	int osrs_t = getOversampling(osrs_tCode);
    	int osrs_p = getOversampling(osrs_pCode);
    	int osrs_h = getOversampling(osrs_hCode);
    	float t_measure_typ = calcTmeas(osrs_t, osrs_p, osrs_h, 2.0, 0.5, 1.0); // msec
    	float t_measure_max = calcTmeas(osrs_t, osrs_p, osrs_h, 2.3, 0.575, 1.25); // msec
    	float t_standby = calcTstandbyMsec(t_standbyCode, isHumidOK);
    	float ODR_typ = 1.0; // Hz
    	const char* modeStr = "SLEEP";
    	const char* calcStr = "";
    	if (forcedMode) {
    		ODR_typ = 1000.0 / (t_measure_typ);
    		modeStr = "FORCED";
    		calcStr = "=1000/t_meas_typ";
    	}
    	if (normalMode) {
    		ODR_typ = 1000.0 / (t_measure_typ + t_standby);
    		modeStr = "NORMAL";
    		calcStr = "=1000/(t_meas_typ+t_standby)";
    	}
    	float t_response75Pct = -1.0; // N.A. if no filtering
    	int nSamps75 = -1;
    	if (filter != 0) {
    		nSamps75 = calcNSamplesTo75Pct(filter);
    		t_response75Pct = nSamps75 * 1000.0 / ODR_typ; // msec
    	}
    	printf("Environmental Sensor Timing:\n-----\n");
    	printf("Operating %s in %s mode.\n", typeStr, modeStr);
    	printf("Oversampling temp %dx, press %dx, humid %dx\n", osrs_t, osrs_p, osrs_h);
    	printf("Measurement time (ms) = %1.2f (typ), %1.2f (max)\n", t_measure_typ, t_measure_max);
    	if (normalMode)
    		printf("Standby time (ms) = %1.2f\n", t_standby);
    	printf("ODR data rate (Hz,%s) = %1.2f\n", calcStr, ODR_typ);
    	if (filter != 0)
    		printf("Filtering w.%d coeffs -- %d samples gets you to 75%% response in t_75resp (ms) = %1.2f\n", getFilterCoeffs(filter), nSamps75, t_response75Pct);
    	else
    		printf("No filtering applied.\n");
    	printf("-----\n");
    }
