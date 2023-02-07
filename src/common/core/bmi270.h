#ifndef SRC_COMMON_HW_INCLUDE_BMI270_H_
#define SRC_COMMON_HW_INCLUDE_BMI270_H_

#include "def.h"
#include "sensor.h"

#define _PIN_DEF_CS 0

bool bmi270_Init(sensor_Dev_t *p_driver);
bool bmi270Detect(uint8_t ch);
bool bmi270SpiAddrRead(void);
bool bmi270SpiAccRead(sensor_Dev_t *p_driver);
bool bmi270SpiGyroRead(sensor_Dev_t *p_driver);
bool bmi270SetCallBack(void (*p_func)(void));

#endif /* SRC_COMMON_HW_INCLUDE_BMI270_H_ */
