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
#include "filter.h"
#include "gyro.h"

#define LPF_MAX_HZ 1000 // so little filtering above 1000hz that if the user wants less delay, they must disable the filter
#define DYN_LPF_MAX_HZ 1000

#define GYRO_LPF1_DYN_MIN_HZ_DEFAULT 250
#define GYRO_LPF1_DYN_MAX_HZ_DEFAULT 500
#define GYRO_LPF2_HZ_DEFAULT 500

#ifdef USE_YAW_SPIN_RECOVERY
#define YAW_SPIN_RECOVERY_THRESHOLD_MIN 500
#define YAW_SPIN_RECOVERY_THRESHOLD_MAX 1950
#endif

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
//typedef struct sensor_Dev_s sensor_Dev_t;
//typedef struct imuSensor_s imuSensor_t;

// typedef struct imuDev_s {
//   uint8_t     gyro_bus_ch;
//   void        (*initFn)(void);
//   bool        (*gyro_readFn)(imuSensor_t *gyro);
//   bool        (*acc_readFn)(imuSensor_t *gyro);
//   bool        (*temp_readFn)(imuSensor_t *gyro);
//   volatile bool dataReady;
//   bool gyroReady;
//   bool accReady;
//   uint32_t targetLooptime;
//   uint32_t sampleLooptime;
//   uint16_t  acc_1G;
//   float     acc_1G_rec;
//   float scale;                                             // scalefactor
//   float gyroZero[XYZ_AXIS_COUNT];
//   float gyroADC[XYZ_AXIS_COUNT];                           // gyro data after calibration and alignment
//   float gyroADCf[XYZ_AXIS_COUNT];    // filtered gyro data
//   uint8_t sampleCount;               // gyro sensor sample counter
//   float sampleSum[XYZ_AXIS_COUNT];   // summed samples used for downsampling
//   int32_t gyroADCRawPrevious[XYZ_AXIS_COUNT];
//   int16_t gyroADCRaw[XYZ_AXIS_COUNT];                      // raw data from sensor
//   bool downsampleFilterEnabled;      // if true then downsample using gyro lowpass 2, otherwise use averaging
//   int16_t accADCRaw[XYZ_AXIS_COUNT];                       // raw data from sensor
//   float   accADC[XYZ_AXIS_COUNT];
//   volatile uint8_t InterruptStatus;
//   bool isAccelUpdatedAtLeastOnce;

//   // lowpass gyro soft filter
//   filterApplyFnPtr lowpassFilterApplyFn;
//   gyroLowpassFilter_t lowpassFilter[XYZ_AXIS_COUNT];

//   // lowpass2 gyro soft filter
//   filterApplyFnPtr lowpass2FilterApplyFn;
//   gyroLowpassFilter_t lowpass2Filter[XYZ_AXIS_COUNT];

//   // notch filters
//   filterApplyFnPtr notchFilter1ApplyFn;
//   biquadFilter_t notchFilter1[XYZ_AXIS_COUNT];

//   filterApplyFnPtr notchFilter2ApplyFn;
//   biquadFilter_t notchFilter2[XYZ_AXIS_COUNT];

//   #ifdef USE_DYN_LPF
//     uint8_t dynLpfFilter;
//     uint16_t dynLpfMin;
//     uint16_t dynLpfMax;
//     uint8_t dynLpfCurveExpo;
// #endif

// #ifdef USE_GYRO_OVERFLOW_CHECK
//     uint8_t overflowAxisMask;
// #endif
// } imuDev_t;

// typedef struct imuCalibration_s
// {
//   float sum[XYZ_AXIS_COUNT];
//   stdev_t var[XYZ_AXIS_COUNT];
//   int32_t cyclesRemaining;
//   uint16_t calibratingA;      // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
//   uint16_t calibratingB;      // baro calibration = get new ground pressure value
//   uint16_t calibratingG;
//   uint8_t gyroCalibrationed;
//   int16_t calibrationCompleted;
//   int16_t accelerationTrims[XYZ_AXIS_COUNT];
// } imuCalibration_t;

// typedef struct imuSensor_s
// {
//   imuDev_t imuDev;
//   imuCalibration_t calibration;
// } imuSensor_t;

// typedef struct sensor_Dev_s
// {
//   float         gyroADC[XYZ_AXIS_COUNT];                          // gyro data after calibration and alignment
//   float         accADC[XYZ_AXIS_COUNT];
//   float         accumulatedMeasurements[XYZ_AXIS_COUNT];
//   int           accumulatedMeasurementCount;
//   imuSensor_t   imuSensor1;
//   int16_t       temperature;
// } sensor_Dev_t;

typedef enum {
    SENSOR_INDEX_GYRO = 0,
    SENSOR_INDEX_ACC,
    SENSOR_INDEX_BARO,
    SENSOR_INDEX_MAG,
    SENSOR_INDEX_RANGEFINDER,
    SENSOR_INDEX_COUNT
} sensorIndex_e;

extern uint8_t requestedSensors[SENSOR_INDEX_COUNT];
extern uint8_t detectedSensors[SENSOR_INDEX_COUNT];

typedef struct int16_flightDynamicsTrims_s {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t calibrationCompleted;
} flightDynamicsTrims_def_t;

typedef union flightDynamicsTrims_u {
    int16_t raw[4];
    flightDynamicsTrims_def_t values;
} flightDynamicsTrims_t;

typedef enum {
    SENSOR_GYRO = 1 << 0, // always present
    SENSOR_ACC = 1 << 1,
    SENSOR_BARO = 1 << 2,
    SENSOR_MAG = 1 << 3,
    SENSOR_SONAR = 1 << 4,
    SENSOR_RANGEFINDER = 1 << 4,
    SENSOR_GPS = 1 << 5,
    SENSOR_GPSMAG = 1 << 6
} sensors_e;

typedef enum {
    GYRO_RATE_1_kHz,
    GYRO_RATE_1100_Hz,
    GYRO_RATE_3200_Hz,
    GYRO_RATE_6400_Hz,
    GYRO_RATE_6664_Hz,
    GYRO_RATE_8_kHz,
    GYRO_RATE_9_kHz,
    GYRO_RATE_32_kHz,
} gyroRateKHz_e;

typedef enum {
    GYRO_EXTI_INIT = 0,
    GYRO_EXTI_INT_DMA,
    GYRO_EXTI_INT,
    GYRO_EXTI_NO_INT
} gyroModeSPI_e;

struct accDev_s;
typedef void (*sensorAccInitFuncPtr)(struct accDev_s *acc);
typedef bool (*sensorAccReadFuncPtr)(struct accDev_s *acc);
struct gyroDev_s;
typedef void (*sensorGyroInitFuncPtr)(struct gyroDev_s *gyro);
typedef bool (*sensorGyroReadFuncPtr)(struct gyroDev_s *gyro);
typedef bool (*sensorGyroReadDataFuncPtr)(struct gyroDev_s *gyro, int16_t *data);

typedef struct gyroDev_s {
    uint8_t  gyro_bus_ch;
    sensorGyroInitFuncPtr initFn;                             // initialize function
    sensorGyroReadFuncPtr readFn;                             // read 3 axis data function
    sensorGyroReadDataFuncPtr temperatureFn;                  // read temperature if available
    //extiCallbackRec_t exti;
    //extDevice_t dev;
    float scale;                                             // scalefactor
    float gyroZero[XYZ_AXIS_COUNT];
    float gyroADC[XYZ_AXIS_COUNT];                           // gyro data after calibration and alignment
    int32_t gyroADCRawPrevious[XYZ_AXIS_COUNT];
    int16_t gyroADCRaw[XYZ_AXIS_COUNT];                      // raw data from sensor
    int16_t temperature;
    //mpuDetectionResult_t mpuDetectionResult;
    //sensor_align_e gyroAlign;
    gyroRateKHz_e gyroRateKHz;
    gyroModeSPI_e gyroModeSPI;
#ifdef USE_GYRO_EXTI
    uint32_t detectedEXTI;
    uint32_t gyroLastEXTI;
    uint32_t gyroSyncEXTI;
    int32_t gyroShortPeriod;
    int32_t gyroDmaMaxDuration;
    busSegment_t segments[2];
#endif
    volatile bool dataReady;
    bool gyro_high_fsr;
    uint8_t hardware_lpf;
    uint8_t hardware_32khz_lpf;
    //uint8_t mpuDividerDrops;
    //ioTag_t mpuIntExtiTag;
    uint8_t gyroHasOverflowProtection;
    //gyroHardware_e gyroHardware;
    fp_rotationMatrix_t rotationMatrix;
    uint16_t gyroSampleRateHz;
    uint16_t accSampleRateHz;
    uint8_t accDataReg;
    uint8_t gyroDataReg;
    volatile uint8_t InterruptStatus;
} gyroDev_t;

typedef struct accDev_s {
    float acc_1G_rec;
    sensorAccInitFuncPtr initFn;                              // initialize function
    sensorAccReadFuncPtr readFn;                              // read 3 axis data function
    uint16_t acc_1G;
    int16_t ADCRaw[XYZ_AXIS_COUNT];
    //mpuDetectionResult_t mpuDetectionResult;
    //sensor_align_e accAlign;
    bool dataReady;
    gyroDev_t *gyro;
    bool acc_high_fsr;
    char revisionCode;                                      // a revision code for the sensor, if known
    uint8_t filler[2];
    //fp_rotationMatrix_t rotationMatrix;
} accDev_t;

bool Sensor_Init(void);
void gyroInitFilters(void);
void accUpdate(uint32_t currentTimeUs);
void imuUpdateAttitude(uint32_t currentTimeUs);
bool accIsCalibrationComplete(void);
void performAcclerationCalibration(void);
void Gyro_getADC(void);
void ACC_getADC(void);
void imuCalculateEstimatedAttitude(uint32_t currentTimeUs);
void DEBUG_print(void);

extern attitudeEulerAngles_t attitude;

#endif /* SRC_COMMON_CORE_SENSOR_H_ */
