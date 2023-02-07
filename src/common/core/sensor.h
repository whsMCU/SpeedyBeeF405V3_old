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

typedef struct sensor_Dev_t_ sensor_Dev_t;

typedef struct mpu_Data_t_
{
  uint16_t x;
  uint16_t y;
  uint16_t z;                                 
} mpu_Data_t;

typedef struct sensor_Dev_t_
{
  void     (*initFn)(void);
  bool     (*gyro_readFn)(sensor_Dev_t *p_driver);
  bool     (*acc_readFn)(sensor_Dev_t *p_driver);
  bool     (*setCallBack)(void (*p_func)(void));
  float    scale; // scalefactor
  mpu_Data_t gyro;
  mpu_Data_t acc;
} sensor_Dev_t;

bool Sensor_Init(void);
void Gyro_getADC(void);
void ACC_getADC(void);

#endif /* SRC_COMMON_CORE_SENSOR_H_ */
