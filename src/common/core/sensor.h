/*
 * sensor.h
 *
 *  Created on: 2020. 12. 20.
 *      Author: WANG
 */

#ifndef SRC_COMMON_CORE_SENSOR_H_
#define SRC_COMMON_CORE_SENSOR_H_


#include "def.h"
#include "axis.h"

#define GYRO_SCALE_2000DPS (2000.0f / (1 << 15))   // 16.384 dps/lsb scalefactor for 2000dps sensors
#define GYRO_SCALE_4000DPS (4000.0f / (1 << 15))   //  8.192 dps/lsb scalefactor for 4000dps sensors

typedef struct sensor_Dev_t_ sensor_Dev_t;

typedef struct sensor_Dev_t_
{
  uint8_t     gyro_bus_ch;
  void        (*initFn)(void);
  bool        (*gyro_readFn)(sensor_Dev_t *gyro);
  bool        (*acc_readFn)(sensor_Dev_t *gyro);
  bool        (*setCallBack)(void (*p_func)(void));
  float       scale; // scalefactor
  float       gyroZero[XYZ_AXIS_COUNT];
  float       gyroADC[XYZ_AXIS_COUNT];                           // gyro data after calibration and alignment
  int32_t     gyroADCRawPrevious[XYZ_AXIS_COUNT];
  int16_t     gyroADCRaw[XYZ_AXIS_COUNT];                      // raw data from sensor
  int16_t     accADCRaw[XYZ_AXIS_COUNT]; 
  int16_t     temperature;
  bool        dataReady;
} sensor_Dev_t;

bool Sensor_Init(void);
void Gyro_getADC(void);
void ACC_getADC(void);

#endif /* SRC_COMMON_CORE_SENSOR_H_ */
