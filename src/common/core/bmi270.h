#ifndef SRC_COMMON_HW_INCLUDE_BMI270_H_
#define SRC_COMMON_HW_INCLUDE_BMI270_H_

#include "def.h"
#include "sensor.h"

#define _PIN_DEF_CS 0

bool bmi270_Init(imuSensor_t *gyro);
bool bmi270Detect(uint8_t ch);
bool bmi270SpiAddrRead(void);
bool bmi270SpiAccRead(imuSensor_t *gyro);
bool bmi270SpiGyroRead(imuSensor_t *gyro);
bool bmi270SetCallBack(void (*p_func)(void));

#endif /* SRC_COMMON_HW_INCLUDE_BMI270_H_ */
