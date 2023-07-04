/*
 * ap.c
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */


#include "ap.h"
#include "uart.h"
#include "scheduler.h"
#include "tasks.h"
#include "rx.h"
#include "pid_init.h"
#include "sensors.h"

  
extern uint32_t _siram_code;
extern uint32_t _sram_code;
extern uint32_t _eram_code;

void ramfuncInit(void)
{
	uint32_t *p_siram_code = &_siram_code;
	uint32_t *p_sram_code  = &_sram_code;
	uint32_t *p_eram_code  = &_eram_code;
	uint32_t lenght;

	lenght = (uint32_t)(p_eram_code - p_sram_code);

	for (int i=0; i<lenght; i++)
	{
		p_sram_code[i] = p_siram_code[i];
	}
}


__attribute__((section(".ram_code"))) void funcRam(void)
{
	volatile float sum, a, b;

	sum = 0.;
	a   = 0.;
	b   = 0.;

	for(int i=0; i<1024*1024; i++)
	{
		sum += a+b;
		a   += 1.;
		b   += 2.;
	}
}

void funcFlash(void)
{
	volatile float sum, a, b;

	sum = 0.;
	a   = 0.;
	b   = 0.;

	for(int i=0; i<1024*1024; i++)
	{
		sum += a+b;
		a   += 1.;
		b   += 2.;
	}
}


void apInit(void)
{
	cliOpen(_DEF_USB, 57600);
	// Initialize task data as soon as possible. Has to be done before tasksInit(),
    // and any init code that may try to modify task behaviour before tasksInit().
    tasksInitData();
	Sensor_Init();
	Baro_Init();
	compassInit();
	gpsInit();
	tasksInit();
	rxInit();
	// Finally initialize the gyro filtering
    gyroInitFilters();
	pidInit();
	ramfuncInit();
}

void apMain(void)
{
	uint32_t pre_time = millis();
	while(1)
	{
		//scheduler();

		if (millis()-pre_time >= 1500)
		{
			pre_time = millis();
			ledToggle(_DEF_LED1);

			uint32_t pre_time_func;
			uint32_t exe_time;

			pre_time_func = millis();
			funcRam();
			exe_time = millis() - pre_time_func;
			cliPrintf("funRam     : %d ms\n", exe_time);

			pre_time_func = millis();
			funcFlash();
			exe_time = millis() - pre_time_func;
			cliPrintf("funFlash   : %d ms\n", exe_time);
		}

	}
	// 	if (millis()-pre_time >= 1000)
    // 	{
    //  		pre_time = millis();
    //   		ledToggle(_DEF_LED1);
    // 	}
	// 	if (micros()-pre_time1 >= 312)
    // 	{
    //  		pre_time1 = micros();
	// 		imuUpdate();	//DT 170us
	// 		baroUpdate(micros());
    // 	}
	// 	if (millis()-pre_time2 >= 50)
    // 	{
    //  		pre_time2 = millis();
	// 		baroUpdate(micros());
	// 		calculateEstimatedAltitude(micros());
    // 	}
	// 	cliMain();
	// }
}
