/*
 * delay.c
 *
 *  Created on: Sep 2, 2016
 *      Author: Mike Mehr
 *
 *  Implementation of Arduino style delay functions for FreeRTOS and/or CMSIS SysTick
 */

#include "FreeRTOS.h"
#include "task.h"


//-------------------------------
// NOT SURE WHAT TO DO WITH THIS BUT IT'S HERE FOR NOW
// If not using FreeRTOS, requires the following code to set up the Systick for 1msec:
/*
int main()
{
 SystemInit(); // or however FreeRTOS wants it...

 SysTick_Config(SystemCoreClock/1000); // 'prescaler' of SysTick for 1 ms

 while(10)
 {
  Delay(100);
 }

}*/
//-------------------------------
//volatile uint32_t msTicks;
//
//void SysTick_Handler (void) //Enter here every 1 ms
//{
//  msTicks++;
//}
void Delay(uint32_t dlyTicksMsec)
{
  uint32_t curTicks;
  uint32_t dlyTicks = dlyTicksMsec * portTICK_PERIOD_MS;

  curTicks = xTaskGetTickCount();
  while ((xTaskGetTickCount() - curTicks) < dlyTicks);
}

void spi_delay(int msec) {
	Delay(msec);
}

uint32_t spi_millis(void) {
	uint32_t msec = xTaskGetTickCount() * portTICK_PERIOD_MS; // convert ticks to msec
	return msec;
}
