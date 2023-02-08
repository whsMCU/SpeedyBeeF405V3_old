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
#include "maths.h"

#define GYRO_SCALE_2000DPS (2000.0f / (1 << 15))   // 16.384 dps/lsb scalefactor for 2000dps sensors
#define GYRO_SCALE_4000DPS (4000.0f / (1 << 15))   //  8.192 dps/lsb scalefactor for 4000dps sensors
typedef struct sensor_Dev_s sensor_Dev_t;
typedef struct imuSensor_s imuSensor_t;

typedef struct imuDev_s {
  uint8_t     gyro_bus_ch;
  void        (*initFn)(void);
  bool        (*gyro_readFn)(imuSensor_t *gyro);
  bool        (*acc_readFn)(imuSensor_t *gyro);
  bool        (*temp_readFn)(imuSensor_t *gyro);
  volatile bool dataReady;
  uint16_t  acc_1G;
  float     acc_1G_rec;
  float scale;                                             // scalefactor
  float gyroZero[XYZ_AXIS_COUNT];
  float gyroADC[XYZ_AXIS_COUNT];                           // gyro data after calibration and alignment
  int32_t gyroADCRawPrevious[XYZ_AXIS_COUNT];
  int16_t gyroADCRaw[XYZ_AXIS_COUNT];                      // raw data from sensor
  int16_t accADCRaw[XYZ_AXIS_COUNT];                       // raw data from sensor
  float   accADC[XYZ_AXIS_COUNT];
} imuDev_t;

typedef struct imuCalibration_s
{
  float sum[XYZ_AXIS_COUNT];
  stdev_t var[XYZ_AXIS_COUNT];
  int32_t cyclesRemaining;
  uint8_t accCalibrationed;
  uint8_t gyroCalibrationed;
} imuCalibration_t;

typedef struct imuSensor_s
{
  imuDev_t imuDev;
  imuCalibration_t calibration;
} imuSensor_t;

typedef struct sensor_Dev_s
{
  float         gyroADC[XYZ_AXIS_COUNT];                          // gyro data after calibration and alignment
  float         accADC[XYZ_AXIS_COUNT];
  imuSensor_t   imuSensor1;
  int16_t       temperature;
} sensor_Dev_t;

bool Sensor_Init(void);
void imuUpdate(void);
void Gyro_getADC(void);
void ACC_getADC(void);

#endif /* SRC_COMMON_CORE_SENSOR_H_ */
