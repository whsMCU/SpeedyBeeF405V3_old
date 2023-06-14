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

typedef struct {
    float w,x,y,z;
} quaternion;
#define QUATERNION_INITIALIZE  {.w=1, .x=0, .y=0,.z=0}

typedef struct {
    float ww,wx,wy,wz,xx,xy,xz,yy,yz,zz;
} quaternionProducts;
#define QUATERNION_PRODUCTS_INITIALIZE  {.ww=1, .wx=0, .wy=0, .wz=0, .xx=0, .xy=0, .xz=0, .yy=0, .yz=0, .zz=0}

typedef union {
    int16_t raw[XYZ_AXIS_COUNT];
    struct {
        // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    } values;
} attitudeEulerAngles_t;

typedef enum {
    GYRO_OVERFLOW_NONE = 0x00,
    GYRO_OVERFLOW_X = 0x01,
    GYRO_OVERFLOW_Y = 0x02,
    GYRO_OVERFLOW_Z = 0x04
} gyroOverflow_e;

#define EULER_INITIALIZE  { { 0, 0, 0 } }

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
  bool gyroReady;
  bool accReady;
  uint32_t targetLooptime;
  uint16_t  acc_1G;
  float     acc_1G_rec;
  float scale;                                             // scalefactor
  float gyroZero[XYZ_AXIS_COUNT];
  float gyroADC[XYZ_AXIS_COUNT];                           // gyro data after calibration and alignment
  int32_t gyroADCRawPrevious[XYZ_AXIS_COUNT];
  int16_t gyroADCRaw[XYZ_AXIS_COUNT];                      // raw data from sensor
  int16_t accADCRaw[XYZ_AXIS_COUNT];                       // raw data from sensor
  float   accADC[XYZ_AXIS_COUNT];
  volatile uint8_t InterruptStatus;
  bool isAccelUpdatedAtLeastOnce;
} imuDev_t;

typedef struct imuCalibration_s
{
  float sum[XYZ_AXIS_COUNT];
  stdev_t var[XYZ_AXIS_COUNT];
  int32_t cyclesRemaining;
  uint16_t calibratingA;      // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
  uint16_t calibratingB;      // baro calibration = get new ground pressure value
  uint16_t calibratingG;
  uint8_t gyroCalibrationed;
  int16_t calibrationCompleted;
  int16_t accelerationTrims[XYZ_AXIS_COUNT];
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
  float         accumulatedMeasurements[XYZ_AXIS_COUNT];
  int           accumulatedMeasurementCount;
  imuSensor_t   imuSensor1;
  int16_t       temperature;
} sensor_Dev_t;

bool Sensor_Init(void);
void gyroUpdate(void);
void accUpdate(uint32_t currentTimeUs);
void imuUpdateAttitude(uint32_t currentTimeUs);
bool accIsCalibrationComplete(void);
void performAcclerationCalibration(void);
void Gyro_getADC(void);
void ACC_getADC(void);
void imuCalculateEstimatedAttitude(uint32_t currentTimeUs);
void DEBUG_print(void);

extern attitudeEulerAngles_t attitude;
extern sensor_Dev_t sensor;

#endif /* SRC_COMMON_CORE_SENSOR_H_ */
