/*
 * sensor.c
 *
 *  Created on: 2020. 12. 20.
 *      Author: WANG
 */

#include "sensor.h"
#include "led.h"
#include "spi.h"
#include "cli.h"

static bool is_init = false;
static sensor_Dev_t sensor;

#ifdef _USE_HW_CLI
static void cliSensor(cli_args_t *args);
#endif

bool Sensor_Init(void)
{
	bool ret = true;
	is_init = bmi270_Init(&sensor);

	if (is_init != true)
 	{
   		return false;
  	}

	#ifdef _USE_HW_CLI
  		cliAdd("Sensor", cliSensor);
	#endif

    return ret;
}

void Gyro_getADC(void)
{

}
void ACC_getADC(void)
{
	
}


#ifdef _USE_HW_CLI
void cliSensor(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "test") == true)
  {
    while(cliKeepLoop())
    {
  
    }
	}

    ret = true;

  if (ret != true)
  {
    cliPrintf("lcd test\n");
    cliPrintf("lcd image\n");
  }
}
#endif