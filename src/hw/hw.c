/*
 * hw.c
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */


#include "hw.h"


void hwInit(void)
{
  bspInit();
  gpioInit();
  usbInit();
  cliInit();
  ledInit();
  uartInit();
  spiInit();
  Sensor_Init();

	// ledOn(ST1);
  // ledOff(ST2);
  // for (int i = 0; i < 10; i++){
  //   ledToggle(ST1);
  //   ledToggle(ST2);
  //   HAL_Delay(25);
  //   //BEEP_ON;
  //   HAL_Delay(25);
  //   //BEEP_OFF;
  // }
  // ledOff(ST1);
  // ledOff(ST2);
}
