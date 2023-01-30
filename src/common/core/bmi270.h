#ifndef SRC_COMMON_HW_INCLUDE_BMI270_H_
#define SRC_COMMON_HW_INCLUDE_BMI270_H_

#include "def.h"
#include "sensor.h"

bool bmi270_Init(sensor_Dev_t *p_driver);
bool bmi270SpiAccRead(void);
bool bmi270SpiGyroRead(void);
bool bmi270SetCallBack(void (*p_func)(void));

#endif /* SRC_COMMON_HW_INCLUDE_BMI270_H_ */
