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
#include "queue.h"
#include "timers.h"
#include "lpc17xx_gpio.h"
/* FreeRTOS+IO includes. */
//#include "FreeRTOS_IO.h"

/* Library includes. */
#include "lpc17xx_gpio.h"

/* Demo includes. */
#include "basic_io.h"
#include "stdio.h"

/* Project includes. */
#include "adc.h"
#include "led.h"
#include "pb.h"
#include "simple7.h"
#include "bmpe.h"

//#define DACOUTPUT // uncomment this line to add support for DAC output for optional ADC testing
#define TEST_SEQUENCES // uncomment this line to add support for sequence testing, such as the 7-segment display
#define DBGPRINTF // uncomment this to have printouts for debugging

#ifdef TEST_SEQUENCES
#define TEST_7SEGMENTS
#endif

#ifdef DBGPRINTF
#define DBGOUTX printf
// uncomment the following to get finer control of printout streams
//#define DBGPRINTLDR
//#define DBGPRINTADC
//#define DBGPRINTPIR
#define DBGPRINTBMPE
//#define DBGPRINT7SEG
//#define DBGPRINTALL
#else
#define DBGOUTX
#endif

#ifdef DBGPRINTLDR
#define DBGOUTLDR DBGOUTX
#else
#define DBGOUTLDR
#endif

#ifdef DBGPRINTPIR
#define DBGOUTXPIR DBGOUTX
#else
#define DBGOUTXPIR
#endif

#ifdef DBGPRINTBMPE
#define DBGOUTXBMPE DBGOUTX
#else
#define DBGOUTXBMPE
#endif

#ifdef DBGPRINTADC
#define DBGOUTXADC DBGOUTX
#else
#define DBGOUTXADC
#endif

#ifdef DBGPRINT7SEG
#define DBGOUTX7SEG DBGOUTX
#else
#define DBGOUTX7SEG
#endif

#ifdef DBGPRINTALL
#define DBGOUTXALL DBGOUTX
#else
#define DBGOUTXALL
#endif

/*-----------------------------------------------------------*/
//  APP OPERATION MODES (PB SELECTABLE STATE TRANSITIONS)
/*-----------------------------------------------------------*/
enum {
	MODE_SMART_BULB,
	MODE_ALWAYS_ON,
	MODE_ALWAYS_OFF,
	MODE_TEST_SEQ,
};

int getNextMode(int curMode, int debug_type) {
	int output;
	int debug = debug_type; // set to 1 to allow always-off mode
	switch (curMode)
	{
	case MODE_SMART_BULB:
		output = MODE_ALWAYS_ON;
		break;
	case MODE_ALWAYS_ON:
		output = debug? MODE_ALWAYS_OFF : MODE_SMART_BULB;
		break;
#ifdef TEST_SEQUENCES
	case MODE_ALWAYS_OFF:
		output = MODE_TEST_SEQ;
		break;
#endif
	default:
		output = MODE_SMART_BULB;
		break;
	}
	return output;
}

/*-----------------------------------------------------------*/
//  COMMAND QUEUE FOR DRIVING THE APP TASK
/*-----------------------------------------------------------*/
QueueHandle_t xCommandQueue = NULL;

#define COMMAND_QUEUE_LENGTH (10)
enum {
	CMD_LDR_DATA = 'A',
	CMD_PIR_DATA,
	CMD_OCC_EVENT,
	CMD_PB_PRESS,
	CMD_BMPE_FIRST_EVENT,
	CMD_BMPE_TEMP_DATA = CMD_BMPE_FIRST_EVENT,
	CMD_BMPE_PRESS_DATA,
	CMD_BMPE_HUMID_DATA,
	CMD_BMPE_ALT_DATA,
};
typedef struct s_Command {
	char code; // use above enum CMD_*
	int data;
} Command;
#define COMMAND_QUEUE_SIZE (sizeof(Command))

/*-----------------------------------------------------------*/
//	DATA QUEUE - EDGE DETECTION FROM DATA STREAM
/*-----------------------------------------------------------*/
QueueHandle_t xEdgeDetectQueue = NULL;
#define EDGE_QUEUE_LENGTH (1)
#define EDGE_QUEUE_SIZE (sizeof(int))

/*-----------------------------------------------------------*/
//	DATA QUEUE - SMART BULB OUTPUT DRIVER
/*-----------------------------------------------------------*/
QueueHandle_t xBulbQueue = NULL;
#define BULB_QUEUE_LENGTH (1)
#define BULB_QUEUE_SIZE (sizeof(int))

/*-----------------------------------------------------------*/
//	DATA QUEUE - SIMPLE SEVEN SEGMENT OUTPUT DRIVER
/*-----------------------------------------------------------*/
QueueHandle_t xSimple7Queue = NULL;
#define SIMPLE7_QUEUE_LENGTH (1)
#define SIMPLE7_QUEUE_SIZE (sizeof(int))

/*-----------------------------------------------------------*/
//	DATA QUEUE - DISPLAY OUTPUT DRIVER
/*-----------------------------------------------------------*/
QueueHandle_t xDisplayQueue = NULL;
#define DISPLAY_QUEUE_LENGTH (1)
typedef struct {
	float temp;
	float press;
	float humid;
	float altitude;
} DisplayResult;
#define DISPLAY_QUEUE_SIZE (sizeof(DisplayResult))

/*-----------------------------------------------------------*/
//  AMBIENT LIGHTING CONVERSION
/*-----------------------------------------------------------*/
#define BRIGHTNESS_MIN (0)
#define BRIGHTNESS_MAX (8)

int invertBrightness(int adcValue) {
	return BRIGHTNESS_MAX - adcValue;
}

// turn an ADC value into a brightness setting
int getAmbientLightingLevel(int adcValue) {
	// interpret the ADC value on a scale from 0 to BRIGHTNESS_MAX
	int brightness = adcScale(adcValue, 0, ADC_MAX_INPUT, 0, BRIGHTNESS_MAX);
	// but due to the circuit used (10k pull-up resistor to 3.3v), higher values mean dimmer so we must invert
	return invertBrightness( brightness );
}

/*-----------------------------------------------------------*/
//  OCCUPANCY CONVERSION (ACTIVITY TIMER)
/*-----------------------------------------------------------*/
/*
 * Ideally, we'd want to timestamp each motion sensor activation, and check for "inactivity" on like a 1-to-5-minute time scale
 * Another way to do this is to fire off a resettable timer every time the rising edge is detected. If the timer ever times out,
 * that would mean inactivity and the occupancy could be set to 0 in that case.
 *
 * */
TimerHandle_t xRoomMotionTimer;

enum {
	OCC_NO_FUNCTION = -1,
	OCC_NONE,
	OCC_ANY,
};

#define ROOM_MOTION_TIMEOUT_MS (1 * 10 * 1000)

static volatile int roomOccupancy = OCC_NO_FUNCTION;

void roomInactivityCallback(TimerHandle_t hTimer) {
	// if this timeout ever occurs, people have left the room due to inactivity
	roomOccupancy = OCC_NONE;
	// we must generate and queue an OCC change event command also
	Command command;
	command.code = CMD_OCC_EVENT;
	command.data = 0; // end of motion; this value will not be sent by the edge detector
	// send the data; by using 0 delay, we just drop data if the queue is full
	xQueueSendToBack(xCommandQueue, &command, 0);
}

int occ_init(void) {
	// create the software timer: 1-shot, callback above
	xRoomMotionTimer = xTimerCreate("Room Motion Timer",
			ROOM_MOTION_TIMEOUT_MS / portTICK_RATE_MS,
			FALSE, // no auto-reload
			NULL, // we don't need the timer ID pointer
			roomInactivityCallback
			);
	// if successful, set occupancy to _NONE, else to _NO_FUNCTION
	if (xRoomMotionTimer == NULL)
		roomOccupancy = OCC_NO_FUNCTION;
	else {
		roomOccupancy = OCC_NONE;
	}
	// returns initial occupancy
	return roomOccupancy;
}

void occ_motion_detected(void) {
	roomOccupancy = OCC_ANY;
	xTimerReset(xRoomMotionTimer, 0);
}

int getOccupancy(void) {
	return roomOccupancy;
}

/*-----------------------------------------------------------*/
//  SMART BULB LOGIC AND OUTPUT
/*-----------------------------------------------------------*/
int getSmartBulbState(int ambientLightLevel, int roomOccupancyLevel) {
	if (roomOccupancyLevel > 0) {
		// if anyone is present in the room, compensate to keep the lighting level high
		// i.e., if the ambient level is lower, set the bulb higher, and vice versa
		// for now, we use the same output range as input ambient range
		return invertBrightness( ambientLightLevel );
	}
	return BRIGHTNESS_MIN;
}

int selectBulbColor(int bulbBrightness, int mode) {
	// this switch statement tries to arrange output colors in order of brightness
	int bulb = bulbBrightness;
	if (mode == MODE_ALWAYS_OFF || mode == MODE_TEST_SEQ)
		bulb = 0;
	if (mode == MODE_ALWAYS_ON)
		bulb = 8;
	switch(bulb) {
	case 0:
		bulb = led2color_black(); break;
	case 1:
		bulb = led2color_red(); break;
	case 2:
		bulb = led2color_green(); break;
	case 3:
		bulb = led2color_blue(); break;
	case 4:
		bulb = led2color_magenta(); break;
	case 5:
		bulb = led2color_yellow(); break;
	case 6:
		bulb = led2color_cyan(); break;
	case 7:
		bulb = led2color_white(); break;
	default: // case 8
		bulb = led2color_white(); break;
	}
	return bulb;
}

/*-----------------------------------------------------------*/
//	TASK - SAMPLE AN ADC CHANNEL, SEND TO QUEUE IF CHANGED
/*-----------------------------------------------------------*/
typedef struct s_SampleParams {
	int channel;
	int sampleRateMsec;
	int cmdCode;
	int lastValue;
} SampleParams;

void vTaskADCInput( void *pvParameters )
{
	SampleParams *params = (SampleParams *)pvParameters;

/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		// read the current value of the ADC
		int value = adc_read_single(params->channel);
		DBGOUTXADC("DEBUG - ADC value %d for ch#%d\n", value, params->channel);

		// only send values that are different from the last sample
		if (params->lastValue != value)
		{
			// save the new value for next time
			params->lastValue = value;
			// send the data to its destination and notify that it is there
			Command command;
			command.code = params->cmdCode;
			command.data = value;
			// send the data; by using 0 delay, we just drop data if the queue is full
			xQueueSendToBack(xCommandQueue, &command, 0);
		}

		// add any optional delay here - otherwise just read and print in a tight loop
		vTaskDelay( params->sampleRateMsec / portTICK_RATE_MS );
	}
}

/*-----------------------------------------------------------*/
//	TASK - DEBOUNCE A GPIO INPUT CHANNEL, SEND TO QUEUE
/*-----------------------------------------------------------*/
void vTaskGPIODebouncer( void *pvParameters )
{
	SampleParams *params = (SampleParams *)pvParameters;

/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		// run the edge detector on the GPIO input bit specified
		int value = pb_detect(params->channel, &params->lastValue);

		if (value != 0) {
			// only send when button-press (+1) or -release (-1) is detected
			// send the data to its destination and notify that it is there
			Command command;
			command.code = params->cmdCode;
			command.data = value;
			// send the data; by using 0 delay, we just drop data if the queue is full
			xQueueSendToBack(xCommandQueue, &command, 0);
		}

		// add any optional delay here - otherwise just read and print in a tight loop
		vTaskDelay( params->sampleRateMsec / portTICK_RATE_MS );
	}
}

/*-----------------------------------------------------------*/
//	TASK - DETECT EDGES IN DATA QUEUE STREAM, SEND TO CMD.QUEUE
/*-----------------------------------------------------------*/
void vTaskEdgeDetector( void *pvParameters )
{
	SampleParams *params = (SampleParams *)pvParameters;
	int edgeThreshold = params->channel;
	int hysteresis = edgeThreshold / 20;
	int edgeThresholdH = edgeThreshold + hysteresis;
	int edgeThresholdL = edgeThreshold - hysteresis;

/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		// read the current value of the input queue (block until ready)
		int data, bit, oldBit;
		xQueueReceive(xEdgeDetectQueue, &data, portMAX_DELAY);
		// apply edge thresholding to generate a bit for debouncing
		// We also need hysteresis using like 5% of the threshold, + and -
		// if current level is above the high threshold, set the output high
		// else if current level is below the low threshold, set the output to low
		// else keep it stable (use the old value stored in LSB of params->lastValue)
		oldBit = params->lastValue & 0b1;
		bit = (data > edgeThresholdH)? 1: (data < edgeThresholdL)? 0: oldBit;

		// run this thru the SW edge detector (looks for N identical bits after a bit change)
		int value = pb_detect_immediate(bit, &params->lastValue);

		if (value != 0) {
			// only send when rising edge (+1) or falling edge (-1) is detected
			// send the data to its destination and notify that it is there
			Command command;
			command.code = params->cmdCode;
			command.data = value;
			// send the data; by using 0 delay, we just drop data if the queue is full
			xQueueSendToBack(xCommandQueue, &command, 0);
		}

		// add any optional delay here - otherwise just run task in a tight loop
		//vTaskDelay( params->sampleRateMsec / portTICK_RATE_MS );
	}
}

/*-----------------------------------------------------------*/
//	TASK - TEMP/HUMIDITY SENSOR INPUT DRIVER (SPI)
/*-----------------------------------------------------------*/
int createBMPEChangeCode(float temp, float press, float alt, float humid) {
	static int val = 1;
	// TODO: convert each float to relevant 8-bit int range, then encode to 32-bit #
	// for now, just assign consecutive numbers so all values get displayed
	return val++;
}

void vTaskBMPEInput( void* pvParameters ) {
	SampleParams *params = (SampleParams *)pvParameters;

/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		// read the current values of the sensor data (temp, press, alt, and poss.humid)
		float temper = bmpe_readTemperature();
		float press = bmpe_readPressure();
		float alt = bmpe_readAltitude();
		float humid = bmpe_readHumidity();
		// create a 4-byte condensed code to detect drastic changes
		int value = createBMPEChangeCode(temper, press, alt, humid); //adc_read_single(params->channel);
		DBGOUTXBMPE("DEBUG - ADC value %d for ch#%d\n", value, params->channel);

		// only send values that are different from the last sample
		if (params->lastValue != value)
		{
			// save the new value for next time
			params->lastValue = value;
			//int currentCode = params->cmdCode;
			// send the data commands to the command queue
			Command command;
			command.code = CMD_BMPE_TEMP_DATA;
			command.data = temper;
			// send the data; by using 0 delay, we just drop data if the queue is full
			xQueueSendToBack(xCommandQueue, &command, 0);
			command.code = CMD_BMPE_PRESS_DATA;
			command.data = press;
			// send the data; by using 0 delay, we just drop data if the queue is full
			xQueueSendToBack(xCommandQueue, &command, 0);
			command.code = CMD_BMPE_HUMID_DATA;
			command.data = humid;
			// send the data; by using 0 delay, we just drop data if the queue is full
			xQueueSendToBack(xCommandQueue, &command, 0);
			// NOTE: Make sure the last event sent (which triggers output) is NOT optional
			command.code = CMD_BMPE_ALT_DATA;
			command.data = alt;
			// send the data; by using 0 delay, we just drop data if the queue is full
			xQueueSendToBack(xCommandQueue, &command, 0);
		}

		// add any optional delay here - otherwise just read and print in a tight loop
		vTaskDelay( params->sampleRateMsec / portTICK_RATE_MS );
	}
}

#ifdef DACOUTPUT
/*-----------------------------------------------------------*/
//	TASK - SEND TRIANGLE WAVE (RAMP) TO DAC
/*-----------------------------------------------------------*/
void vTaskDACOutput( void *pvParameters )
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
//		DBGOUTX( "***>>> Task 2: DAC setting = %d/1024 = %d/4096, output (mv) = %d\n", value, value << 2, mv );

		// output an ever-increasing value to the DAC (will be converted into a 10-bit fraction internally by bit masking)
		// This should output a stair-step wave on AOUT at a frequency set by the delay value
		dac_out(value);
		ul += 64;


	}
}
#endif

/*-----------------------------------------------------------*/
//	TASK - DATA CONCENTRATOR (CMD.QUEUE READ AND PROCESS)
/*-----------------------------------------------------------*/
void vTaskDataConcentrator( void *pvParameters )
{
	// set up variables for current settings
	int ldrReading = 0; // LDR raw ADC reading
	int ldrmv = 0; // mv of same (DEBUG)
	int ambient = 0;  // ambient lighting level
	int pirReading = 0; // PIR raw ADC reading
	int pirmv = 0; // mv of same (DEBUG)
	int occupancy = 1; // room occupancy level
	int brightness = 0; // Smart Bulb brightness setting
	int mode = MODE_SMART_BULB; // application operation mode (PB setting)
	int temperatureReading = 0; // from the optional BMPE sensor
	int pressureReading = 0; // from the BMPE sensor
	int altitudeReading = 0; // from the BMPE sensor
	int humidityReading = 0; // from the BMPE sensor

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		// read command/data from queue and process it with output to display(s)
		Command currentCmd;
		xQueueReceive(xCommandQueue, &currentCmd, portMAX_DELAY);
		int updated = FALSE; // updates needed to go to common indicators (LED, 7SEG)
		int updatedEnv = FALSE; // update needed to go to optional display (OLED)

		switch (currentCmd.code)
		{
		case CMD_LDR_DATA:
			// interpret the ADC value as DARK or LIGHT brightness level (actually many levels now)
			ldrReading = currentCmd.data;
			ambient = getAmbientLightingLevel(ldrReading);
			updated = TRUE;
			break;
		case CMD_PIR_DATA:
			// interpret the ADC value as a change in room occupancy level (one person leaving/entering for now)
			//occupancy = 1; // version overriding sensor
			pirReading = currentCmd.data;
			pirmv = frac2MV(pirReading, ADC_MAX_INPUT);
			DBGOUTXPIR( "PIR=%d, mv=%d\n", pirReading, pirmv );
			xQueueOverwrite(xEdgeDetectQueue, &pirReading);
			break;
		case CMD_OCC_EVENT:
			// the data sent by the edge detector is 1 or -1; 0 will be sent by the timeout command (occupancy has changed)
			if (currentCmd.data != 0)
				occ_motion_detected(); // generate the occupancy change if edge event (motion)
			occupancy = getOccupancy();
			DBGOUTXPIR( "**   OCC=%d   **\n", occupancy );
			updated = TRUE;
			break;
		case CMD_PB_PRESS:
			if (currentCmd.data < 0)
				break; // ignore button releases
			mode = getNextMode(mode, 1); // 2nd param includes total shutoff for debug purposes
			updated = TRUE;
			break;
		case CMD_BMPE_TEMP_DATA:
			// interpret the sensor values as appropriate (integer assumed to have N decimals
			temperatureReading = currentCmd.data;
			// intermediate data value - no update yet
			break;
		case CMD_BMPE_PRESS_DATA:
			// interpret the sensor values as appropriate (integer assumed to have N decimals
			pressureReading = currentCmd.data;
			// intermediate data value - no update yet
			break;
		case CMD_BMPE_HUMID_DATA:
			// interpret the sensor values as appropriate (integer assumed to have N decimals
			humidityReading = currentCmd.data;
			// intermediate data value - no update yet
			break;
		case CMD_BMPE_ALT_DATA:
			// interpret the sensor values as appropriate (integer assumed to have N decimals
			altitudeReading = currentCmd.data;
			// since this is the last of four always sent together, allow env.display updates now
			updatedEnv = TRUE;
			break;
		default:
			break;
		}

		if (updated) {
			// if any of the following are updated: ambient, occupancy, mode
			// then we need to send the output to the Bulb/LED, 7SEG, and DEBUG queues/tasks

			// determine the smart bulb state from provided conditions (ambient lighting, room occupancy)
			brightness = getSmartBulbState( ambient, occupancy );
			// once the brightness level is known, we can set the on-board LED color with the current mode
			int bulbColor = selectBulbColor(brightness, mode);
			// send this data to the LED output queue(s)
			xQueueOverwrite(xBulbQueue, &bulbColor); // LED gets smart bulb output level

			// process the conditions (ambient lighting, room occupancy) into proper output for the 7SEG display
			extern int getAmbientDisplayCode(int ambient, int occupancy, int mode);
			int codedAmbient = getAmbientDisplayCode(ambient, occupancy, mode);
			// send this data to the 7SEG output queue
			xQueueOverwrite(xSimple7Queue, &codedAmbient);

			// DEBUG: no separate task, just output immediately
			/* Print out the name of this task and the current values read. */
			ldrmv = frac2MV(ldrReading, ADC_MAX_INPUT);
			DBGOUTXALL( "Data task: LDR reading=%d/4096, input (mv)=%d, ambient=%d, occup=%d, BRT=%d, MODE=%d, color=%d\n",
					ldrReading, ldrmv, ambient, occupancy, brightness, mode, bulbColor );
		}

		if (updatedEnv) {
			// if we have changes in relevant data, update the environment display (OLED)
			// Now Showing: temperature, pressure, humidity, altitude (abs/rel)
			DisplayResult resData;
			resData.temp = temperatureReading;
			resData.press = pressureReading;
			resData.humid = humidityReading;
			resData.altitude = altitudeReading;
			//resData.refAlt = altitudeReference;
			// send this data to the OLED output queue(s)
			xQueueOverwrite(xDisplayQueue, &resData);
		}

	}
}

/*-----------------------------------------------------------*/
//	TASK - SMART BULB OUTPUT DRIVER
/*-----------------------------------------------------------*/
void vTaskLEDOutput( void *pvParameters )
{
	/* As per most tasks, this task is implemented in an infinite loop. */

	for( ;; )
	{
		// read data from queue and process it with output to led
		int data;
		xQueueReceive(xBulbQueue, &data, portMAX_DELAY);

		// process data with output to onboard LED
		led2_set(data);
	}
}

/*-----------------------------------------------------------*/
//	TASK - SIMPLE SEVEN-SEGMENT DISPLAY OUTPUT DRIVER
/*-----------------------------------------------------------*/
int getAmbientDisplayCode(int ambient, int occupancy, int mode) {
	int output = ambient; // since we can't negate 0
	// this will turn on the decimal point if room is occupied
	output += (occupancy == 0)? 0: s7_getDataRange();
#ifdef TEST_7SEGMENTS
	if (mode == MODE_TEST_SEQ) {
		// test mode: special code to kick off the 7SEG Test task sequence
		// this sequence ends when normal data is written, i.e., when we exit the test mode
		output = -1;
	}
#endif
	return output;
}

#ifdef TEST_7SEGMENTS
#define TEST_SEQ_DELAY (1000 / portTICK_RATE_MS)

void programTests(int testProgram[], int* finalIndex, int* finalHexIndex, int* finalTestIndex) {
	int index = 0;
	int hexTestMax = s7_getDataRange();
	int testMax = s7_getNumberOfTestPatterns();
	int testProgramMax = 0;
	// program hex font tests (32)
	// all hex digits
	for (index=0; index<hexTestMax; ++index) {
		testProgram[testProgramMax++] = index;
	}
	// all hex digits w.decimal point
	for (index=0; index<hexTestMax; ++index) {
		testProgram[testProgramMax++] = index + hexTestMax;
	}
	// save high water mark so far
	*finalHexIndex = testProgramMax;
	// program test font tests
	for (index=0; index<testMax; ++index) {
		testProgram[testProgramMax++] = index;
	}
	// save high water mark so far
	*finalTestIndex = testProgramMax;
	// program ascii font tests (29)
	const char* txt = "C9876543210.,dDeEaAdDbBeEeEfF";
	for (index=0; index<29; ++index) {
		testProgram[testProgramMax++] = txt[index];
	}
	// save high water mark so far
	*finalIndex = testProgramMax;
}
#endif

void vTaskSimple7Output( void *pvParameters )
{
	// One-time setup for test sequence.
	int testData = 0;
#ifdef TEST_7SEGMENTS
	static int testProgram[100]; // room for expansion
	int testProgramMax, useTestAfter, useAsciiAfter;
	programTests(testProgram, &testProgramMax, &useTestAfter, &useAsciiAfter);
#endif

	/* As per most tasks, this task is implemented in an infinite loop. */

	for( ;; )
	{
		// read data from queue and process it with output to led
		int data;
		xQueueReceive(xSimple7Queue, &data, portMAX_DELAY);

		// process data with output to onboard LED
		if (data >= 0) {
			s7_writeBinaryDP(data);
			DBGOUTX7SEG("7SEG = %d\n", data);
		}
#ifdef TEST_7SEGMENTS
		else {
			// special code for test sequence
			testData = 0;
			for( ;; )
			{
				// peek data from queue to detect return to normal data (>=0)
				data = -1; // in case of timeout, stay in test sequence mode
				// also if data changes happen, the queue still is set to -1, so we stay in test seq.mode
				int iPeekResult = xQueuePeek(xSimple7Queue, &data, 0);

				if (data >= 0) {
					// return to normal loop processing
					break;
				}
				else {
					// if a -1 was in the queue, read it before returning to peek/timeout test
					if (iPeekResult == pdTRUE) {
						// read without delay, and ignore the received data (it will be -1)
						xQueueReceive(xSimple7Queue, &data, 0);
					}
					// write the next test element in the sequence pattern
					data = testProgram[testData];
					if (testData >= useAsciiAfter)
						s7_writeAscii(data, 0);
					else if (testData >= useTestAfter)
						s7_writeTest(data);
					else
						s7_writeBinaryDP(data);
					// index the test sequence pattern, with wraparound
					++testData;
					if (testData >= testProgramMax)
						testData = 0;
					// wait for a timeout (ignores data during this time, so make it short I suppose)
					vTaskDelay(TEST_SEQ_DELAY);
					// now, back to the peek loop to check for sequence commands (real data or -1 or timeouts)
				}
			}
		}
#endif
	}
}

/*-----------------------------------------------------------*/
//	TASK - OLED DISPLAY (I2C) OUTPUT DRIVER
/*-----------------------------------------------------------*/
void vTaskOLEDOutput( void* pvParameters ) {
	/* As per most tasks, this task is implemented in an infinite loop. */

	for( ;; )
	{
		// read data from queue and process it with output to led
		DisplayResult data;
		xQueueReceive(xDisplayQueue, &data, portMAX_DELAY);

		// process data with output to OLED multiline display
		DBGOUTXBMPE("Temperature(deg.C): %d\nPressure(hPa): %d\nHumidity(?): %d\nAltitude(m): %d\n",
				data.temp, data.press, data.humid, data.altitude);
	}
}

/*-----------------------------------------------------------*/
#define LDR_CHANNEL (0) // ADC input channel 0-7
#define LDR_SAMPLE_RATE_MS (250)
SampleParams ldrParams;

#define GPIO_CHANNEL (9) // GPIO pin number code
#define GPIO_SAMPLE_RATE_MS (4)
SampleParams gpioParams;

#define PIR_CHANNEL (1) // ADC input channel 0-7
#define PIR_SAMPLE_RATE_MS (85) // needs to generate >10 levels before state change, or use different edge detector
SampleParams pirParams;

#define EDGE_THRESHOLD (1000) // ADC signal threshold (counts)
#define EDGE_SAMPLE_RATE_MS (0)
SampleParams pirEdgeParams;

#define BMPE_CHANNEL (0) // SPI channel 0 with SSEL0
#define BMPE_SAMPLE_RATE_MS (5000)
SampleParams bmpeParams;

int bmpeType = BMPE_NONE; // global feature: what kind of environment sensor
int initProgress = 0; // debug code

int main( void )
{
	/* Init the semi-hosting. */
	DBGOUTX( "\n" );
	// init the ADC system (for LDR reading) and DAC (for testing)
	adc_init();
#ifdef DACOUTPUT
	dac_init();
	dac_out(1023);
#endif
	// init the onboard LED system (for Smart Bulb output)
	led2_init();
	led2_set(led2color_black());
	// init the PB system
	pb_init(GPIO_CHANNEL);
	// init the edge detection system
	occ_init();
	// init the 7-segment LED GPIO pins
	s7_init();
	// init the BMP/BME sensor system
	bmpeType = bmpe_init();
	switch (bmpeType) {
	case BMPE_BMPTYPE:
		DBGOUTXBMPE("Using BMP temperature/pressure sensor.\n");
		break;
	case BMPE_BMETYPE:
		DBGOUTXBMPE("Using BMP temperature/pressure/humidity sensor.\n");
		break;
	}
	initProgress = 1;

	// set up the command and data queues
	xCommandQueue = xQueueCreate( COMMAND_QUEUE_LENGTH, COMMAND_QUEUE_SIZE );
	initProgress = 2;
	xEdgeDetectQueue = xQueueCreate( EDGE_QUEUE_LENGTH, EDGE_QUEUE_SIZE );
	xBulbQueue = xQueueCreate( BULB_QUEUE_LENGTH, BULB_QUEUE_SIZE );
	xSimple7Queue = xQueueCreate( SIMPLE7_QUEUE_LENGTH, SIMPLE7_QUEUE_SIZE );
	xDisplayQueue = xQueueCreate( DISPLAY_QUEUE_LENGTH, DISPLAY_QUEUE_SIZE );
	initProgress = 3;
	int tester = TRUE;
	tester = tester && (xCommandQueue != NULL);
	tester = tester && (xEdgeDetectQueue != NULL);
	tester = tester && (xBulbQueue != NULL);
	tester = tester && (xSimple7Queue != NULL);
	tester = tester && (xDisplayQueue != NULL);
	if (tester) {

		/* Create input tasks. */
		ldrParams.channel = LDR_CHANNEL;
		ldrParams.cmdCode = CMD_LDR_DATA;
		ldrParams.sampleRateMsec = LDR_SAMPLE_RATE_MS;
		ldrParams.lastValue = -1;
		xTaskCreate(	vTaskADCInput,		/* Pointer to the function that implements the task. */
						"LDR Input",	/* Text name for the task.  This is to facilitate debugging only. */
						120,		/* Stack depth in words. */
						(void *) &ldrParams,		/* Pass the param pointer in the task parameter. */
						2,			/* This task will run at priority X. */
						NULL );		/* We are not using the task handle. */
		initProgress = 100;

		gpioParams.channel = GPIO_CHANNEL;
		gpioParams.cmdCode = CMD_PB_PRESS;
		gpioParams.sampleRateMsec = GPIO_SAMPLE_RATE_MS;
		gpioParams.lastValue = 0;
		xTaskCreate(	vTaskGPIODebouncer,		/* Pointer to the function that implements the task. */
						"PB Input",	/* Text name for the task.  This is to facilitate debugging only. */
						120,		/* Stack depth in words. */
						(void *) &gpioParams,		/* Pass the param pointer in the task parameter. */
						2,			/* This task will run at priority X. */
						NULL );		/* We are not using the task handle. */
		initProgress = 200;

		pirParams.channel = PIR_CHANNEL;
		pirParams.cmdCode = CMD_PIR_DATA;
		pirParams.sampleRateMsec = PIR_SAMPLE_RATE_MS;
		pirParams.lastValue = 0;
		xTaskCreate(	vTaskADCInput,		/* Pointer to the function that implements the task. */
						"PIR Input",	/* Text name for the task.  This is to facilitate debugging only. */
						120,		/* Stack depth in words. */
						(void *) &pirParams,		/* Pass the param pointer in the task parameter. */
						2,			/* This task will run at priority X. */
						NULL );		/* We are not using the task handle. */
		initProgress = 300;

		pirEdgeParams.channel = EDGE_THRESHOLD;
		pirEdgeParams.cmdCode = CMD_OCC_EVENT;
		pirEdgeParams.sampleRateMsec = EDGE_SAMPLE_RATE_MS;
		pirEdgeParams.lastValue = 0;
		xTaskCreate(	vTaskEdgeDetector,		/* Pointer to the function that implements the task. */
						"PIR Edges",	/* Text name for the task.  This is to facilitate debugging only. */
						120,		/* Stack depth in words. */
						(void *) &pirEdgeParams,		/* Pass the param pointer in the task parameter. */
						2,			/* This task will run at priority X. */
						NULL );		/* We are not using the task handle. */
		initProgress = 400;

		if (bmpeType != BMPE_NONE) {
			// if there's no BMPE sensor, no need to measure anything there
			bmpeParams.channel = BMPE_CHANNEL;
			bmpeParams.cmdCode = CMD_BMPE_FIRST_EVENT;
			bmpeParams.sampleRateMsec = BMPE_SAMPLE_RATE_MS;
			bmpeParams.lastValue = 0;
			xTaskCreate(	vTaskBMPEInput,		/* Pointer to the function that implements the task. */
							"BMPE Sensors",	/* Text name for the task.  This is to facilitate debugging only. */
							240,		/* Stack depth in words. */
							(void *) &bmpeParams,		/* Pass the param pointer in the task parameter. */
							2,			/* This task will run at priority X. */
							NULL );		/* We are not using the task handle. */
			initProgress = 500;
		}

#ifdef DACOUTPUT
		xTaskCreate(	vTaskDACOutput,		/* Pointer to the function that implements the task. */
						"Test Output",	/* Text name for the task.  This is to facilitate debugging only. */
						240,		/* Stack depth in words. */
						NULL,		/* We are not using the task parameter. */
						1,			/* This task will run at priority Y. */
						NULL );		/* We are not using the task handle. */
		initProgress = 600;
#endif

		xTaskCreate(	vTaskDataConcentrator,		/* Pointer to the function that implements the task. */
						"Data Concentrator",	/* Text name for the task.  This is to facilitate debugging only. */
						240,		/* Stack depth in words. */
						NULL,		/* We are not using the task parameter. */
						1,			/* This task will run at priority Z. */
						NULL );		/* We are not using the task handle. */
		initProgress = 700;

		xTaskCreate(	vTaskLEDOutput,		/* Pointer to the function that implements the task. */
						"LED Output",	/* Text name for the task.  This is to facilitate debugging only. */
						120,		/* Stack depth in words. */
						NULL,		/* We are not using the task parameter. */
						1,			/* This task will run at priority A. */
						NULL );		/* We are not using the task handle. */
		initProgress = 800;

		xTaskCreate(	vTaskSimple7Output,		/* Pointer to the function that implements the task. */
						"SSEG Output",	/* Text name for the task.  This is to facilitate debugging only. */
						120,		/* Stack depth in words. */
						NULL,		/* We are not using the task parameter. */
						1,			/* This task will run at priority A. */
						NULL );		/* We are not using the task handle. */
		initProgress = 900;

		xTaskCreate(	vTaskOLEDOutput,		/* Pointer to the function that implements the task. */
						"OLED Output",	/* Text name for the task.  This is to facilitate debugging only. */
						240,		/* Stack depth in words. */
						NULL,		/* We are not using the task parameter. */
						1,			/* This task will run at priority A. */
						NULL );		/* We are not using the task handle. */
		initProgress = 1000;


		/* Start the scheduler so our tasks start executing. */
		vTaskStartScheduler();
		initProgress = 1100;

	}


	/* If all is well we will never reach here as the scheduler will now be
	running.  If we do reach here then it is likely that there was insufficient
	heap available for the idle task to be created. */
	for( ;; );
	return 0;
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* This function will only be called if an API call to create a task, queue
	or semaphore fails because there is too little heap RAM remaining. */
	DBGOUTX("Allocation failed error, code=%d.\n", initProgress);
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will only be called if a task overflows its stack.  Note
	that stack overflow checking does slow down the context switch
	implementation. */
	DBGOUTX("Stack overflow error.\n");
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
