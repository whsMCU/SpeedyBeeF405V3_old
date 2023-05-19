/*
 * hw.h
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */

#ifndef SRC_HW_HW_H_
#define SRC_HW_HW_H_

#include "hw_def.h"

#include "led.h"
#include "uart.h"
#include "usb.h"
#include "cli.h"
#include "i2c.h"
#include "timer.h"
#include "spi.h"
#include "gpio.h"
#include "sensor.h"
#include "bmi270.h"
#include "barometer.h"
#include "barometer_dps310.h"
#include "compass.h"
#include "compass_qmc5883l.h"
#include "gps.h"


void hwInit(void);


#endif /* SRC_HW_HW_H_ */
