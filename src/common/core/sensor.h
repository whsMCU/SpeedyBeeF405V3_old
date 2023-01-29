/*
 * sensor.h
 *
 *  Created on: 2020. 12. 20.
 *      Author: WANG
 */

#ifndef SRC_COMMON_CORE_SENSOR_H_
#define SRC_COMMON_CORE_SENSOR_H_


#include "def.h"

#define GYRO_SCALE_2000DPS (2000.0f / (1 << 15))   // 16.384 dps/lsb scalefactor for 2000dps sensors
#define GYRO_SCALE_4000DPS (4000.0f / (1 << 15))   //  8.192 dps/lsb scalefactor for 4000dps sensors

typedef struct sensor_Dev_t sensor_Dev_t;

typedef struct sensor_Dev_t
{
  bool     (*initFn)(void);
  bool     (*gyro_readFn)(void);
  bool     (*acc_readFn)(void);
  bool     (*setCallBack)(void (*p_func)(void));
  float scale;                                             // scalefactor
  float gyroZero[3];
  float gyroADC[3];                           // gyro data after calibration and alignment
  int32_t gyroADCRawPrevious[3];
  int16_t gyroADCRaw[3];  

} sensor_Dev_t;

bool Sensor_Init(void);
void Gyro_getADC(void);
void ACC_getADC(void);

#endif /* SRC_COMMON_CORE_SENSOR_H_ */
