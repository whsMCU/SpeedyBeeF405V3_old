/*
 * ap.c
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */


#include "ap.h"
#include "uart.h"

void apInit(void)
{
	cliOpen(_DEF_USB, 57600);

	Sensor_Init();
	Baro_Init();
}

void apMain(void)
{
	uint32_t pre_time, pre_time1;
 	pre_time = millis();
	pre_time1 = micros();
	while(1)
	{
		if (millis()-pre_time >= 100)
    	{
     		pre_time = millis();
      		ledToggle(_DEF_LED1);
			DEBUG_print();
    	}
		if (micros()-pre_time1 >= 312)
    	{
     		pre_time1 = micros();
			imuUpdate();	//DT 170us
			baroUpdate();
    	}
		cliMain();
	}
}
