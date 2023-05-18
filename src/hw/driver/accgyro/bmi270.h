#ifndef SRC_COMMON_HW_INCLUDE_BMI270_H_
#define SRC_COMMON_HW_INCLUDE_BMI270_H_

#include "def.h"
#include "sensor.h"

bool bmi270_Init(imuSensor_t *gyro);
bool bmi270Detect(uint8_t ch);
bool bmi270SpiAccRead(imuSensor_t *gyro);
bool bmi270SpiGyroRead(imuSensor_t *gyro);
bool bmi270SetCallBack(void (*p_func)(void));
uint8_t bmi270InterruptStatus(imuSensor_t *gyro);

#endif /* SRC_COMMON_HW_INCLUDE_BMI270_H_ */
