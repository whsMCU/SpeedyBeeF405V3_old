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
}

void apMain(void)
{
	uint32_t pre_time;
 	pre_time = millis();
	while(1)
	{
		if (millis()-pre_time >= 1000)
    	{
     		pre_time = millis();
      		ledToggle(_DEF_LED1);
    	}
		cliMain();
	}
}
