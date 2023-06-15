/*
 * sensor.c
 *
 *  Created on: 2020. 12. 20.
 *      Author: WANG
 */

#include "sensor.h"
#include "driver/accgyro/bmi270.h"
#include "cli.h"
#include "led.h"
#include "filter.h"

static inline int32_t cmpTimeUs(uint32_t a, uint32_t b) { return (int32_t)(a - b); }

static bool is_init = false;

#define CALIBRATING_ACC_CYCLES              512
#define acc_lpf_factor 4

#define dcm_kp 2500                // 1.0 * 10000
#define dcm_ki 0                   // 0.003 * 10000
#define small_angle 25

#define SPIN_RATE_LIMIT 20

#define ATTITUDE_RESET_QUIET_TIME 250000   // 250ms - gyro quiet period after disarm before attitude reset
#define ATTITUDE_RESET_GYRO_LIMIT 15       // 15 deg/sec - gyro limit for quiet period
#define ATTITUDE_RESET_KP_GAIN    25.0     // dcmKpGain value to use during attitude reset
#define ATTITUDE_RESET_ACTIVE_TIME 500000  // 500ms - Time to wait for attitude to converge at high gain
#define GPS_COG_MIN_GROUNDSPEED 500        // 500cm/s minimum groundspeed for a gps heading to be considered valid

float accAverage[XYZ_AXIS_COUNT];

uint8_t activePidLoopDenom = 2;

static bool overflowDetected;
#ifdef USE_GYRO_OVERFLOW_CHECK
static uint32_t overflowTimeUs;
#endif

#ifdef USE_YAW_SPIN_RECOVERY
static bool yawSpinRecoveryEnabled;
static int yawSpinRecoveryThreshold;
static bool yawSpinDetected;
static uint32_t yawSpinTimeUs;
#endif

#define GYRO_OVERFLOW_TRIGGER_THRESHOLD 31980  // 97.5% full scale (1950dps for 2000dps gyro)
#define GYRO_OVERFLOW_RESET_THRESHOLD 30340    // 92.5% full scale (1850dps for 2000dps gyro)

static float gyro_accumulatedMeasurements[XYZ_AXIS_COUNT];
static float gyroPrevious[XYZ_AXIS_COUNT];
static int gyro_accumulatedMeasurementCount;

float rMat[3][3];

static bool attitudeIsEstablished = false;

// quaternion of sensor frame relative to earth frame
static quaternion q = QUATERNION_INITIALIZE;
static quaternionProducts qP = QUATERNION_PRODUCTS_INITIALIZE;
// headfree quaternions
quaternion headfree = QUATERNION_INITIALIZE;
quaternion offset = QUATERNION_INITIALIZE;

// absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
attitudeEulerAngles_t attitude = EULER_INITIALIZE;

sensor_Dev_t sensor;

#ifdef _USE_HW_CLI
static void cliSensor(cli_args_t *args);
#endif

static void imuQuaternionComputeProducts(quaternion *quat, quaternionProducts *quatProd)
{
    quatProd->ww = quat->w * quat->w;
    quatProd->wx = quat->w * quat->x;
    quatProd->wy = quat->w * quat->y;
    quatProd->wz = quat->w * quat->z;
    quatProd->xx = quat->x * quat->x;
    quatProd->xy = quat->x * quat->y;
    quatProd->xz = quat->x * quat->z;
    quatProd->yy = quat->y * quat->y;
    quatProd->yz = quat->y * quat->z;
    quatProd->zz = quat->z * quat->z;
}

static void imuComputeRotationMatrix(void)
{
    imuQuaternionComputeProducts(&q, &qP);

    rMat[0][0] = 1.0f - 2.0f * qP.yy - 2.0f * qP.zz;
    rMat[0][1] = 2.0f * (qP.xy + -qP.wz);
    rMat[0][2] = 2.0f * (qP.xz - -qP.wy);

    rMat[1][0] = 2.0f * (qP.xy - -qP.wz);
    rMat[1][1] = 1.0f - 2.0f * qP.xx - 2.0f * qP.zz;
    rMat[1][2] = 2.0f * (qP.yz + -qP.wx);

    rMat[2][0] = 2.0f * (qP.xz + -qP.wy);
    rMat[2][1] = 2.0f * (qP.yz - -qP.wx);
    rMat[2][2] = 1.0f - 2.0f * qP.xx - 2.0f * qP.yy;
}

static void Calibrate_gyro(void)
{
	int cal_int = 0;
	uint8_t axis = 0;

	for (cal_int = 0; cal_int < 2000; cal_int ++)
	{
		if (cal_int % 500 == 0)
		{
            ledToggle(_DEF_LED1); //Change the led status to indicate calibration.
		}
		Gyro_getADC();
		for(axis=0; axis<3; axis++)
		{
      sensor.imuSensor1.calibration.sum[axis] += (float)sensor.imuSensor1.imuDev.gyroADCRaw[axis];
		}
	}
	for(axis=0; axis<3; axis++)
	{
    sensor.imuSensor1.imuDev.gyroZero[axis] = sensor.imuSensor1.calibration.sum[axis]/2000.0f;
	}
	HAL_Delay(100);
}

bool Sensor_Init(void)
{
	bool ret = true;
	is_init = bmi270_Init(&sensor.imuSensor1);
  
    Calibrate_gyro();
	if (is_init != true)
 	{
   		return false;
  	}
    sensor.imuSensor1.imuDev.checkOverflow = GYRO_OVERFLOW_CHECK_ALL_AXES;
    #ifdef USE_GYRO_OVERFLOW_CHECK
    if (sensor.imuSensor1.imuDev.checkOverflow == GYRO_OVERFLOW_CHECK_YAW) {
        sensor.imuSensor1.imuDev.overflowAxisMask = GYRO_OVERFLOW_Z;
    } else if (sensor.imuSensor1.imuDev.checkOverflow == GYRO_OVERFLOW_CHECK_ALL_AXES) {
        sensor.imuSensor1.imuDev.overflowAxisMask = GYRO_OVERFLOW_X | GYRO_OVERFLOW_Y | GYRO_OVERFLOW_Z;
    } else {
        sensor.imuSensor1.imuDev.overflowAxisMask = 0;
    }
    #endif
    sensor.imuSensor1.imuDev.gyroHasOverflowProtection = true;

	#ifdef _USE_HW_CLI
  		cliAdd("Sensor", cliSensor);
	#endif

    return ret;
}

static void applyAccelerationTrims(const sensor_Dev_t *accelerationTrims)
{
    sensor.accADC[X] -= sensor.imuSensor1.calibration.accelerationTrims[X];
    sensor.accADC[Y] -= sensor.imuSensor1.calibration.accelerationTrims[Y];
    sensor.accADC[Z] -= sensor.imuSensor1.calibration.accelerationTrims[Z];
}

#ifdef USE_GYRO_OVERFLOW_CHECK
static void handleOverflow(uint32_t currentTimeUs)
{
    // This will need to be revised if we ever allow different sensor types to be
    // used simultaneously. In that case the scale might be different between sensors.
    // It's complicated by the fact that we're using filtered gyro data here which is
    // after both sensors are scaled and averaged.
    const float gyroOverflowResetRate = GYRO_OVERFLOW_RESET_THRESHOLD * GYRO_SCALE_2000DPS;

    if ((fabsf(sensor.imuSensor1.imuDev.gyroADCf[X]) < gyroOverflowResetRate)
          && (fabsf(sensor.imuSensor1.imuDev.gyroADCf[Y]) < gyroOverflowResetRate)
          && (fabsf(sensor.imuSensor1.imuDev.gyroADCf[Z]) < gyroOverflowResetRate)) {
        // if we have 50ms of consecutive OK gyro vales, then assume yaw readings are OK again and reset overflowDetected
        // reset requires good OK values on all axes
        if (cmpTimeUs(currentTimeUs, overflowTimeUs) > 50000) {
            overflowDetected = false;
        }
    } else {
        // not a consecutive OK value, so reset the overflow time
        overflowTimeUs = currentTimeUs;
    }
}

static void checkForOverflow(uint32_t currentTimeUs)
{
    // check for overflow to handle Yaw Spin To The Moon (YSTTM)
    // ICM gyros are specified to +/- 2000 deg/sec, in a crash they can go out of spec.
    // This can cause an overflow and sign reversal in the output.
    // Overflow and sign reversal seems to result in a gyro value of +1996 or -1996.
    if (overflowDetected) {
        handleOverflow(currentTimeUs);
    } else {
#ifndef SIMULATOR_BUILD
        // check for overflow in the axes set in overflowAxisMask
        gyroOverflow_e overflowCheck = GYRO_OVERFLOW_NONE;

        // This will need to be revised if we ever allow different sensor types to be
        // used simultaneously. In that case the scale might be different between sensors.
        // It's complicated by the fact that we're using filtered gyro data here which is
        // after both sensors are scaled and averaged.
        const float gyroOverflowTriggerRate = GYRO_OVERFLOW_TRIGGER_THRESHOLD * GYRO_SCALE_2000DPS;

        if (fabsf(sensor.imuSensor1.imuDev.gyroADCf[X]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_X;
        }
        if (fabsf(sensor.imuSensor1.imuDev.gyroADCf[Y]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_Y;
        }
        if (fabsf(sensor.imuSensor1.imuDev.gyroADCf[Z]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_Z;
        }
        if (overflowCheck & sensor.imuSensor1.imuDev.overflowAxisMask) {
            overflowDetected = true;
            overflowTimeUs = currentTimeUs;
#ifdef USE_YAW_SPIN_RECOVERY
            yawSpinDetected = false;
#endif // USE_YAW_SPIN_RECOVERY
        }
#endif // SIMULATOR_BUILD
    }
}
#endif // USE_GYRO_OVERFLOW_CHECK

#ifdef USE_YAW_SPIN_RECOVERY
static void handleYawSpin(uint32_t currentTimeUs)
{
    const float yawSpinResetRate = yawSpinRecoveryThreshold - 100.0f;
    if (fabsf(sensor.imuSensor1.imuDev.gyroADCf[Z]) < yawSpinResetRate) {
        // testing whether 20ms of consecutive OK gyro yaw values is enough
        if (cmpTimeUs(currentTimeUs, yawSpinTimeUs) > 20000) {
            yawSpinDetected = false;
        }
    } else {
        // reset the yaw spin time
        yawSpinTimeUs = currentTimeUs;
    }
}

static void checkForYawSpin(uint32_t currentTimeUs)
{
    // if not in overflow mode, handle yaw spins above threshold
#ifdef USE_GYRO_OVERFLOW_CHECK
    if (overflowDetected) {
        yawSpinDetected = false;
        return;
    }
#endif // USE_GYRO_OVERFLOW_CHECK

    if (yawSpinDetected) {
        handleYawSpin(currentTimeUs);
    } else {
#ifndef SIMULATOR_BUILD
        // check for spin on yaw axis only
         if (abs((int)sensor.imuSensor1.imuDev.gyroADCf[Z]) > yawSpinRecoveryThreshold) {
            yawSpinDetected = true;
            yawSpinTimeUs = currentTimeUs;
        }
#endif // SIMULATOR_BUILD
    }
}
#endif // USE_YAW_SPIN_RECOVERY

static bool gyroInitLowpassFilterLpf(int slot, int type, uint16_t lpfHz, uint32_t looptime)
{
    filterApplyFnPtr *lowpassFilterApplyFn;
    gyroLowpassFilter_t *lowpassFilter = NULL;

    switch (slot) {
    case FILTER_LPF1:
        lowpassFilterApplyFn = &sensor.imuSensor1.imuDev.lowpassFilterApplyFn;
        lowpassFilter = sensor.imuSensor1.imuDev.lowpassFilter;
        break;

    case FILTER_LPF2:
        lowpassFilterApplyFn = &sensor.imuSensor1.imuDev.lowpass2FilterApplyFn;
        lowpassFilter = sensor.imuSensor1.imuDev.lowpass2Filter;
        break;

    default:
        return false;
    }

    bool ret = false;

    // Establish some common constants
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / looptime;
    const float gyroDt = looptime * 1e-6f;

    // Gain could be calculated a little later as it is specific to the pt1/bqrcf2/fkf branches
    const float gain = pt1FilterGain(lpfHz, gyroDt);

    // Dereference the pointer to null before checking valid cutoff and filter
    // type. It will be overridden for positive cases.
    *lowpassFilterApplyFn = nullFilterApply;

    // If lowpass cutoff has been specified
    if (lpfHz) {
        switch (type) {
        case FILTER_PT1:
            *lowpassFilterApplyFn = (filterApplyFnPtr) pt1FilterApply;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterInit(&lowpassFilter[axis].pt1FilterState, gain);
            }
            ret = true;
            break;
        case FILTER_BIQUAD:
            if (lpfHz <= gyroFrequencyNyquist) {
#ifdef USE_DYN_LPF
                *lowpassFilterApplyFn = (filterApplyFnPtr) biquadFilterApplyDF1;
#else
                *lowpassFilterApplyFn = (filterApplyFnPtr) biquadFilterApply;
#endif
                for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                    biquadFilterInitLPF(&lowpassFilter[axis].biquadFilterState, lpfHz, looptime);
                }
                ret = true;
            }
            break;
        case FILTER_PT2:
            *lowpassFilterApplyFn = (filterApplyFnPtr) pt2FilterApply;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt2FilterInit(&lowpassFilter[axis].pt2FilterState, gain);
            }
            ret = true;
            break;
        case FILTER_PT3:
            *lowpassFilterApplyFn = (filterApplyFnPtr) pt3FilterApply;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt3FilterInit(&lowpassFilter[axis].pt3FilterState, gain);
            }
            ret = true;
            break;
        }
    }
    return ret;
}

static uint16_t calculateNyquistAdjustedNotchHz(uint16_t notchHz, uint16_t notchCutoffHz)
{
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / sensor.imuSensor1.imuDev.targetLooptime;
    if (notchHz > gyroFrequencyNyquist) {
        if (notchCutoffHz < gyroFrequencyNyquist) {
            notchHz = gyroFrequencyNyquist;
        } else {
            notchHz = 0;
        }
    }

    return notchHz;
}

static void gyroInitFilterNotch1(uint16_t notchHz, uint16_t notchCutoffHz)
{
    sensor.imuSensor1.imuDev.notchFilter1ApplyFn = nullFilterApply;

    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);

    if (notchHz != 0 && notchCutoffHz != 0) {
        sensor.imuSensor1.imuDev.notchFilter1ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&sensor.imuSensor1.imuDev.notchFilter1[axis], notchHz, sensor.imuSensor1.imuDev.targetLooptime, notchQ, FILTER_NOTCH, 1.0f);
        }
    }
}

static void gyroInitFilterNotch2(uint16_t notchHz, uint16_t notchCutoffHz)
{
    sensor.imuSensor1.imuDev.notchFilter2ApplyFn = nullFilterApply;

    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);

    if (notchHz != 0 && notchCutoffHz != 0) {
        sensor.imuSensor1.imuDev.notchFilter2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&sensor.imuSensor1.imuDev.notchFilter2[axis], notchHz, sensor.imuSensor1.imuDev.targetLooptime, notchQ, FILTER_NOTCH, 1.0f);
        }
    }
}

#ifdef USE_DYN_LPF
static void dynLpfFilterInit()
{
    if (GYRO_LPF1_DYN_MIN_HZ_DEFAULT > 0) {
        switch (FILTER_PT1) {
        case FILTER_PT1:
            sensor.imuSensor1.imuDev.dynLpfFilter = DYN_LPF_PT1;
            break;
        case FILTER_BIQUAD:
            sensor.imuSensor1.imuDev.dynLpfFilter = DYN_LPF_BIQUAD;
            break;
        case FILTER_PT2:
            sensor.imuSensor1.imuDev.dynLpfFilter = DYN_LPF_PT2;
            break;
        case FILTER_PT3:
            sensor.imuSensor1.imuDev.dynLpfFilter = DYN_LPF_PT3;
            break;
        default:
            sensor.imuSensor1.imuDev.dynLpfFilter = DYN_LPF_NONE;
            break;
        }
    } else {
        sensor.imuSensor1.imuDev.dynLpfFilter = DYN_LPF_NONE;
    }
    sensor.imuSensor1.imuDev.dynLpfMin = GYRO_LPF1_DYN_MIN_HZ_DEFAULT;
    sensor.imuSensor1.imuDev.dynLpfMax = GYRO_LPF1_DYN_MAX_HZ_DEFAULT;
    sensor.imuSensor1.imuDev.dynLpfCurveExpo = 5;
}
#endif

void gyroInitFilters(void)
{
    uint16_t gyro_lpf1_init_hz = GYRO_LPF1_DYN_MIN_HZ_DEFAULT;//gyroConfig()->gyro_lpf1_static_hz;

#ifdef USE_DYN_LPF
    if (GYRO_LPF1_DYN_MIN_HZ_DEFAULT > 0) {
        gyro_lpf1_init_hz = GYRO_LPF1_DYN_MIN_HZ_DEFAULT;
    }
#endif

    gyroInitLowpassFilterLpf(
      FILTER_LPF1,
      FILTER_PT1,
      gyro_lpf1_init_hz,
      sensor.imuSensor1.imuDev.targetLooptime
    );

    sensor.imuSensor1.imuDev.downsampleFilterEnabled = gyroInitLowpassFilterLpf(
      FILTER_LPF2,
      FILTER_PT1,
      GYRO_LPF2_HZ_DEFAULT,
      sensor.imuSensor1.imuDev.sampleLooptime
    );

    gyroInitFilterNotch1(0, 0);//gyroConfig()->gyro_soft_notch_hz_1, gyroConfig()->gyro_soft_notch_cutoff_1);
    gyroInitFilterNotch2(0, 0);//gyroConfig()->gyro_soft_notch_hz_2, gyroConfig()->gyro_soft_notch_cutoff_2);
#ifdef USE_DYN_LPF
    dynLpfFilterInit();
#endif
#ifdef USE_DYN_NOTCH_FILTER
    dynNotchInit(dynNotchConfig(), gyro.targetLooptime);
#endif
}

void gyroUpdate(void)
{
    //static float gyroLPF[3];

    imuSensor_t *imu = &sensor.imuSensor1;
    imu->imuDev.InterruptStatus = bmi270InterruptStatus(imu);

    if(imu->imuDev.InterruptStatus & 0x40)
    {
        imu->imuDev.gyroReady = true;
    }

    if(imu->imuDev.InterruptStatus & 0x80)
    {
        imu->imuDev.accReady = true;
    }
    
    if(imu->imuDev.gyroReady)
    {
        imu->imuDev.gyro_readFn(&sensor.imuSensor1);
        imu->imuDev.gyroADC[X] = imu->imuDev.gyroADCRaw[X] - imu->imuDev.gyroZero[X];
        imu->imuDev.gyroADC[Y] = imu->imuDev.gyroADCRaw[Y] - imu->imuDev.gyroZero[Y];
        imu->imuDev.gyroADC[Z] = imu->imuDev.gyroADCRaw[Z] - imu->imuDev.gyroZero[Z];
        imu->imuDev.gyroReady = false;
    }
    imu->imuDev.dataReady = false;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
    {
        sensor.gyroADC[axis] = sensor.imuSensor1.imuDev.gyroADC[axis] * sensor.imuSensor1.imuDev.scale;
    }

    if (sensor.imuSensor1.imuDev.downsampleFilterEnabled) {
    // using gyro lowpass 2 filter for downsampling
    sensor.imuSensor1.imuDev.sampleSum[X] = sensor.imuSensor1.imuDev.lowpass2FilterApplyFn((filter_t *)&sensor.imuSensor1.imuDev.lowpass2Filter[X], sensor.gyroADC[X]);
    sensor.imuSensor1.imuDev.sampleSum[Y] = sensor.imuSensor1.imuDev.lowpass2FilterApplyFn((filter_t *)&sensor.imuSensor1.imuDev.lowpass2Filter[Y], sensor.gyroADC[Y]);
    sensor.imuSensor1.imuDev.sampleSum[Z] = sensor.imuSensor1.imuDev.lowpass2FilterApplyFn((filter_t *)&sensor.imuSensor1.imuDev.lowpass2Filter[Z], sensor.gyroADC[Z]);
    } else {
    // using simple averaging for downsampling
    sensor.imuSensor1.imuDev.sampleSum[X] += sensor.gyroADC[X];
    sensor.imuSensor1.imuDev.sampleSum[Y] += sensor.gyroADC[Y];
    sensor.imuSensor1.imuDev.sampleSum[Z] += sensor.gyroADC[Z];
    sensor.imuSensor1.imuDev.sampleCount++;
    }

    // for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
    // {
    //     // integrate using trapezium rule to avoid bias
    //     gyro_accumulatedMeasurements[axis] += 0.5f * (gyroPrevious[axis] + sensor.gyroADC[axis]) * sensor.imuSensor1.imuDev.targetLooptime;
    //     gyroPrevious[axis] = sensor.gyroADC[axis];
    // }
    // gyro_accumulatedMeasurementCount++;
}

#define GYRO_FILTER_FUNCTION_NAME filterGyro
#define GYRO_FILTER_DEBUG_SET(mode, index, value) do { UNUSED(mode); UNUSED(index); UNUSED(value); } while (0)
#define GYRO_FILTER_AXIS_DEBUG_SET(axis, mode, index, value) do { UNUSED(axis); UNUSED(mode); UNUSED(index); UNUSED(value); } while (0)
#include "gyro_filter_impl.c"
#undef GYRO_FILTER_FUNCTION_NAME
#undef GYRO_FILTER_DEBUG_SET
#undef GYRO_FILTER_AXIS_DEBUG_SET

// #define GYRO_FILTER_FUNCTION_NAME filterGyroDebug
// #define GYRO_FILTER_DEBUG_SET DEBUG_SET
// #define GYRO_FILTER_AXIS_DEBUG_SET(axis, mode, index, value) if (axis == (int)gyro.gyroDebugAxis) DEBUG_SET(mode, index, value)
// #include "gyro_filter_impl.c"
// #undef GYRO_FILTER_FUNCTION_NAME
// #undef GYRO_FILTER_DEBUG_SET
// #undef GYRO_FILTER_AXIS_DEBUG_SET

void gyroFiltering(uint32_t currentTimeUs)
{
    //if (gyro.gyroDebugMode == DEBUG_NONE) {
        filterGyro();
    // } else {
    //     filterGyroDebug();
    // }

#ifdef USE_DYN_NOTCH_FILTER
    if (isDynNotchActive()) {
        dynNotchUpdate();
    }
#endif

//     if (gyro.useDualGyroDebugging) {
//         switch (gyro.gyroToUse) {
//         case GYRO_CONFIG_USE_GYRO_1:
//             DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 0, gyro.gyroSensor1.gyroDev.gyroADCRaw[X]);
//             DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 1, gyro.gyroSensor1.gyroDev.gyroADCRaw[Y]);
//             DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 0, lrintf(gyro.gyroSensor1.gyroDev.gyroADC[X] * gyro.gyroSensor1.gyroDev.scale));
//             DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 1, lrintf(gyro.gyroSensor1.gyroDev.gyroADC[Y] * gyro.gyroSensor1.gyroDev.scale));
//             break;

// #ifdef USE_MULTI_GYRO
//         case GYRO_CONFIG_USE_GYRO_2:
//             DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 2, gyro.gyroSensor2.gyroDev.gyroADCRaw[X]);
//             DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 3, gyro.gyroSensor2.gyroDev.gyroADCRaw[Y]);
//             DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 2, lrintf(gyro.gyroSensor2.gyroDev.gyroADC[X] * gyro.gyroSensor2.gyroDev.scale));
//             DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 3, lrintf(gyro.gyroSensor2.gyroDev.gyroADC[Y] * gyro.gyroSensor2.gyroDev.scale));
//             break;

//     case GYRO_CONFIG_USE_GYRO_BOTH:
//             DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 0, gyro.gyroSensor1.gyroDev.gyroADCRaw[X]);
//             DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 1, gyro.gyroSensor1.gyroDev.gyroADCRaw[Y]);
//             DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 2, gyro.gyroSensor2.gyroDev.gyroADCRaw[X]);
//             DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 3, gyro.gyroSensor2.gyroDev.gyroADCRaw[Y]);
//             DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 0, lrintf(gyro.gyroSensor1.gyroDev.gyroADC[X] * gyro.gyroSensor1.gyroDev.scale));
//             DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 1, lrintf(gyro.gyroSensor1.gyroDev.gyroADC[Y] * gyro.gyroSensor1.gyroDev.scale));
//             DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 2, lrintf(gyro.gyroSensor2.gyroDev.gyroADC[X] * gyro.gyroSensor2.gyroDev.scale));
//             DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 3, lrintf(gyro.gyroSensor2.gyroDev.gyroADC[Y] * gyro.gyroSensor2.gyroDev.scale));
//             DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 0, lrintf((gyro.gyroSensor1.gyroDev.gyroADC[X] * gyro.gyroSensor1.gyroDev.scale) - (gyro.gyroSensor2.gyroDev.gyroADC[X] * gyro.gyroSensor2.gyroDev.scale)));
//             DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 1, lrintf((gyro.gyroSensor1.gyroDev.gyroADC[Y] * gyro.gyroSensor1.gyroDev.scale) - (gyro.gyroSensor2.gyroDev.gyroADC[Y] * gyro.gyroSensor2.gyroDev.scale)));
//             DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 2, lrintf((gyro.gyroSensor1.gyroDev.gyroADC[Z] * gyro.gyroSensor1.gyroDev.scale) - (gyro.gyroSensor2.gyroDev.gyroADC[Z] * gyro.gyroSensor2.gyroDev.scale)));
//             break;
// #endif
//         }
//     }

#ifdef USE_GYRO_OVERFLOW_CHECK
    if (sensor.imuSensor1.imuDev.checkOverflow && !sensor.imuSensor1.imuDev.gyroHasOverflowProtection) {
        checkForOverflow(currentTimeUs);
    }
#endif

#ifdef USE_YAW_SPIN_RECOVERY
    if (yawSpinRecoveryEnabled) {
        checkForYawSpin(currentTimeUs);
    }
#endif

    if (!overflowDetected) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            // integrate using trapezium rule to avoid bias
            gyro_accumulatedMeasurements[axis] += 0.5f * (gyroPrevious[axis] + sensor.imuSensor1.imuDev.gyroADCf[axis]) * sensor.imuSensor1.imuDev.targetLooptime;
            gyroPrevious[axis] = sensor.imuSensor1.imuDev.gyroADCf[axis];
        }
        gyro_accumulatedMeasurementCount++;
    }

#if !defined(USE_GYRO_OVERFLOW_CHECK) && !defined(USE_YAW_SPIN_RECOVERY)
    UNUSED(currentTimeUs);
#endif
}

void accUpdate(uint32_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    static float accLPF[3];
    imuSensor_t *imu = &sensor.imuSensor1;

    if(imu->imuDev.accReady)
    {
        imu->imuDev.acc_readFn(&sensor.imuSensor1);
        imu->imuDev.isAccelUpdatedAtLeastOnce = true;
        imu->imuDev.accADC[X] = imu->imuDev.accADCRaw[X];
        imu->imuDev.accADC[Y] = imu->imuDev.accADCRaw[Y];
        imu->imuDev.accADC[Z] = imu->imuDev.accADCRaw[Z];
        imu->imuDev.accReady = false;
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
    {
        sensor.accADC[axis] = imu->imuDev.accADC[axis];
    }
    for(int axis=0;axis<3;axis++)
	{
		if (acc_lpf_factor > 0)
		{
			accLPF[axis] = accLPF[axis] * (1.0f - (1.0f / acc_lpf_factor)) + sensor.accADC[axis] * (1.0f / acc_lpf_factor);
			sensor.accADC[axis] = accLPF[axis];
		}
	}

    if (!accIsCalibrationComplete()) {
        performAcclerationCalibration();
    }

    applyAccelerationTrims(&sensor);

    ++sensor.accumulatedMeasurementCount;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        sensor.accumulatedMeasurements[axis] += sensor.accADC[axis];
    }
}

void imuUpdateAttitude(uint32_t currentTimeUs)
{
    if (sensor.imuSensor1.imuDev.isAccelUpdatedAtLeastOnce) {

        imuCalculateEstimatedAttitude(currentTimeUs);
    }
}

bool accGetAccumulationAverage(float *accumulationAverage)
{
  if (sensor.accumulatedMeasurementCount > 0) {
    // If we have gyro data accumulated, calculate average rate that will yield the same rotation
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
      accumulationAverage[axis] = sensor.accumulatedMeasurements[axis] / sensor.accumulatedMeasurementCount;
      sensor.accumulatedMeasurements[axis] = 0.0f;
    }
    sensor.accumulatedMeasurementCount = 0;
    return true;
    } else {
      for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        accumulationAverage[axis] = 0.0f;
      }
      return false;
    }
}

bool accIsCalibrationComplete(void)
{
    return sensor.imuSensor1.calibration.calibratingA == 0;
}

static bool isOnFinalAccelerationCalibrationCycle(void)
{
    return sensor.imuSensor1.calibration.calibratingA == 1;
}

static bool isOnFirstAccelerationCalibrationCycle(void)
{
    return sensor.imuSensor1.calibration.calibratingA == CALIBRATING_ACC_CYCLES;
}

static void setConfigCalibrationCompleted(void)
{
    sensor.imuSensor1.calibration.calibrationCompleted = 1;
}

void performAcclerationCalibration(void)
{
    static int32_t a[3];

    for (int axis = 0; axis < 3; axis++) {

        // Reset a[axis] at start of calibration
        if (isOnFirstAccelerationCalibrationCycle()) {
            a[axis] = 0;
        }

        // Sum up CALIBRATING_ACC_CYCLES readings
        a[axis] += sensor.accADC[axis];

        // Reset global variables to prevent other code from using un-calibrated data
        sensor.accADC[axis] = 0;
        sensor.imuSensor1.calibration.accelerationTrims[axis] = 0;
    }

    if (isOnFinalAccelerationCalibrationCycle()) {
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        sensor.imuSensor1.calibration.accelerationTrims[X] = (a[X] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        sensor.imuSensor1.calibration.accelerationTrims[Y] = (a[Y] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        sensor.imuSensor1.calibration.accelerationTrims[Z] = (a[Z] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - sensor.imuSensor1.imuDev.acc_1G;

        setConfigCalibrationCompleted();

        //saveConfigAndNotify();
    }

    sensor.imuSensor1.calibration.calibratingA--;
}

void Gyro_getADC(void)
{
  bmi270SpiGyroRead(&sensor.imuSensor1);
  //cliPrintf("gyro x: %d, y: %d, z: %d\n\r", sensor.imuSensor1.imuDev.gyroADCRaw[X], sensor.imuSensor1.imuDev.gyroADCRaw[Y], sensor.imuSensor1.imuDev.gyroADCRaw[Z]);
}
void ACC_getADC(void)
{
  bmi270SpiAccRead(&sensor.imuSensor1);
  //cliPrintf("acc x: %d, y: %d, z: %d\n\r", sensor.imuSensor1.imuDev.accADCRaw[X], sensor.imuSensor1.imuDev.accADCRaw[Y], sensor.imuSensor1.imuDev.accADCRaw[Z]);
}

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

static void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz,
                                bool useAcc, float ax, float ay, float az,
                                bool useMag,
                                bool useCOG, float courseOverGround, const float dcmKpGain)
{
    static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;    // integral error terms scaled by Ki

    // Calculate general spin rate (rad/s)
    const float spin_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));

    // Use raw heading error (from GPS or whatever else)
    float ex = 0, ey = 0, ez = 0;
    if (useCOG) {
        while (courseOverGround >  M_PIf) {
            courseOverGround -= (2.0f * M_PIf);
        }

        while (courseOverGround < -M_PIf) {
            courseOverGround += (2.0f * M_PIf);
        }

        const float ez_ef = (- sin_approx(courseOverGround) * rMat[0][0] - cos_approx(courseOverGround) * rMat[1][0]);

        ex = rMat[2][0] * ez_ef;
        ey = rMat[2][1] * ez_ef;
        ez = rMat[2][2] * ez_ef;
    }

#ifdef USE_MAG
    // Use measured magnetic field vector
    float mx = mag.magADC[X];
    float my = mag.magADC[Y];
    float mz = mag.magADC[Z];
    float recipMagNorm = sq(mx) + sq(my) + sq(mz);
    if (useMag && recipMagNorm > 0.01f) {
        // Normalise magnetometer measurement
        recipMagNorm = invSqrt(recipMagNorm);
        mx *= recipMagNorm;
        my *= recipMagNorm;
        mz *= recipMagNorm;

        // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
        // This way magnetic field will only affect heading and wont mess roll/pitch angles

        // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
        // (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
        const float hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
        const float hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
        const float bx = sqrtf(hx * hx + hy * hy);

        // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
        const float ez_ef = -(hy * bx);

        // Rotate mag error vector back to BF and accumulate
        ex += rMat[2][0] * ez_ef;
        ey += rMat[2][1] * ez_ef;
        ez += rMat[2][2] * ez_ef;
    }
#else
    UNUSED(useMag);
#endif

    // Use measured acceleration vector
    float recipAccNorm = sq(ax) + sq(ay) + sq(az);
    if (useAcc && recipAccNorm > 0.01f) {
        // Normalise accelerometer measurement
        recipAccNorm = invSqrt(recipAccNorm);
        ax *= recipAccNorm;
        ay *= recipAccNorm;
        az *= recipAccNorm;

        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex += (ay * rMat[2][2] - az * rMat[2][1]);
        ey += (az * rMat[2][0] - ax * rMat[2][2]);
        ez += (ax * rMat[2][1] - ay * rMat[2][0]);
    }

    // Compute and apply integral feedback if enabled
    if (dcm_ki > 0.0f) {
        // Stop integrating if spinning beyond the certain limit
        if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
            const float dcmKiGain = dcm_ki;
            integralFBx += dcmKiGain * ex * dt;    // integral error scaled by Ki
            integralFBy += dcmKiGain * ey * dt;
            integralFBz += dcmKiGain * ez * dt;
        }
    } else {
        integralFBx = 0.0f;    // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // Apply proportional and integral feedback
    gx += dcmKpGain * ex + integralFBx;
    gy += dcmKpGain * ey + integralFBy;
    gz += dcmKpGain * ez + integralFBz;

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    quaternion buffer;
    buffer.w = q.w;
    buffer.x = q.x;
    buffer.y = q.y;
    buffer.z = q.z;

    q.w += (-buffer.x * gx - buffer.y * gy - buffer.z * gz);
    q.x += (+buffer.w * gx + buffer.y * gz - buffer.z * gy);
    q.y += (+buffer.w * gy - buffer.x * gz + buffer.z * gx);
    q.z += (+buffer.w * gz + buffer.x * gy - buffer.y * gx);

    // Normalise quaternion
    float recipNorm = invSqrt(sq(q.w) + sq(q.x) + sq(q.y) + sq(q.z));
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;

    // Pre-compute rotation matrix from quaternion
    imuComputeRotationMatrix();

    attitudeIsEstablished = true;
}

static bool imuIsAccelerometerHealthy(float *accAverage)
{
    float accMagnitudeSq = 0;
    for (int axis = 0; axis < 3; axis++) {
        const float a = accAverage[axis];
        accMagnitudeSq += a * a;
    }

    accMagnitudeSq = accMagnitudeSq * sq(sensor.imuSensor1.imuDev.acc_1G_rec);

    // Accept accel readings only in range 0.9g - 1.1g
    return (0.81f < accMagnitudeSq) && (accMagnitudeSq < 1.21f);
}

static bool gyroGetAccumulationAverage(float *accumulationAverage)
{
    if (gyro_accumulatedMeasurementCount) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        const uint32_t accumulatedMeasurementTimeUs = gyro_accumulatedMeasurementCount * sensor.imuSensor1.imuDev.targetLooptime;//gyro.targetLooptime;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = gyro_accumulatedMeasurements[axis] / accumulatedMeasurementTimeUs;
            gyro_accumulatedMeasurements[axis] = 0.0f;
        }
        gyro_accumulatedMeasurementCount = 0;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}

// Calculate the dcmKpGain to use. When armed, the gain is imuRuntimeConfig.dcm_kp * 1.0 scaling.
// When disarmed after initial boot, the scaling is set to 10.0 for the first 20 seconds to speed up initial convergence.
// After disarming we want to quickly reestablish convergence to deal with the attitude estimation being incorrect due to a crash.
//   - wait for a 250ms period of low gyro activity to ensure the craft is not moving
//   - use a large dcmKpGain value for 500ms to allow the attitude estimate to quickly converge
//   - reset the gain back to the standard setting
static float imuCalcKpGain(uint32_t currentTimeUs, bool useAcc, float *gyroAverage)
{
  static bool lastArmState = false;
  static uint32_t gyroQuietPeriodTimeEnd = 0;
  static uint32_t attitudeResetTimeEnd = 0;
  static bool attitudeResetCompleted = false;
  float ret;
  bool attitudeResetActive = false;

  const bool armState = false;//ARMING_FLAG(ARMED);

  if (!armState) {
      if (lastArmState) {   // Just disarmed; start the gyro quiet period
          gyroQuietPeriodTimeEnd = currentTimeUs + ATTITUDE_RESET_QUIET_TIME;
          attitudeResetTimeEnd = 0;
          attitudeResetCompleted = false;
      }

      // If gyro activity exceeds the threshold then restart the quiet period.
      // Also, if the attitude reset has been complete and there is subsequent gyro activity then
      // start the reset cycle again. This addresses the case where the pilot rights the craft after a crash.
      if ((attitudeResetTimeEnd > 0) || (gyroQuietPeriodTimeEnd > 0) || attitudeResetCompleted) {
          if ((fabsf(gyroAverage[X]) > ATTITUDE_RESET_GYRO_LIMIT)
              || (fabsf(gyroAverage[Y]) > ATTITUDE_RESET_GYRO_LIMIT)
              || (fabsf(gyroAverage[Z]) > ATTITUDE_RESET_GYRO_LIMIT)
              || (!useAcc)) {

              gyroQuietPeriodTimeEnd = currentTimeUs + ATTITUDE_RESET_QUIET_TIME;
              attitudeResetTimeEnd = 0;
          }
      }
      if (attitudeResetTimeEnd > 0) {        // Resetting the attitude estimation
          if (currentTimeUs >= attitudeResetTimeEnd) {
              gyroQuietPeriodTimeEnd = 0;
              attitudeResetTimeEnd = 0;
              attitudeResetCompleted = true;
          } else {
              attitudeResetActive = true;
          }
      } else if ((gyroQuietPeriodTimeEnd > 0) && (currentTimeUs >= gyroQuietPeriodTimeEnd)) {
          // Start the high gain period to bring the estimation into convergence
          attitudeResetTimeEnd = currentTimeUs + ATTITUDE_RESET_ACTIVE_TIME;
          gyroQuietPeriodTimeEnd = 0;
      }
  }
  lastArmState = armState;

  if (attitudeResetActive) {
      ret = ATTITUDE_RESET_KP_GAIN;
  } else {
      ret = 0.25;
      if (!armState) {
        ret = ret * 10.0f; // Scale the kP to generally converge faster when disarmed.
      }
  }

  return ret;
}

static void imuUpdateEulerAngles(void)
{
    quaternionProducts buffer;

    if (false) { //FLIGHT_MODE(HEADFREE_MODE)
       imuQuaternionComputeProducts(&headfree, &buffer);

       attitude.values.roll = lrintf(atan2_approx((+2.0f * (buffer.wx + buffer.yz)), (+1.0f - 2.0f * (buffer.xx + buffer.yy))) * (1800.0f / M_PIf));
       attitude.values.pitch = lrintf(((0.5f * M_PIf) - acos_approx(+2.0f * (buffer.wy - buffer.xz))) * (1800.0f / M_PIf));
       attitude.values.yaw = lrintf((-atan2_approx((+2.0f * (buffer.wz + buffer.xy)), (+1.0f - 2.0f * (buffer.yy + buffer.zz))) * (1800.0f / M_PIf)));
    } else {
       attitude.values.roll = lrintf(atan2_approx(rMat[2][1], rMat[2][2]) * (1800.0f / M_PIf));
       attitude.values.pitch = lrintf(((0.5f * M_PIf) - acos_approx(-rMat[2][0])) * (1800.0f / M_PIf));
       attitude.values.yaw = lrintf((-atan2_approx(rMat[1][0], rMat[0][0]) * (1800.0f / M_PIf)));
    }

    if (attitude.values.yaw < 0) {
        attitude.values.yaw += 3600;
    }
}

void imuCalculateEstimatedAttitude(uint32_t currentTimeUs)
{
    static uint32_t previousIMUUpdateTime;
    bool useAcc = false;
    bool useMag = false;
    bool useCOG = false; // Whether or not correct yaw via imuMahonyAHRSupdate from our ground course
    float courseOverGround = 0; // To be used when useCOG is true.  Stored in Radians

    const int32_t deltaT = currentTimeUs - previousIMUUpdateTime;
    previousIMUUpdateTime = currentTimeUs;

    #ifdef USE_MAG
    if (sensors(SENSOR_MAG) && compassIsHealthy())
    {
        useMag = true;
    }
    #endif

    #if defined(USE_GPS)
    if (!useMag && sensors(SENSOR_GPS) && STATE(GPS_FIX) && gpsSol.numSat >= 5 && gpsSol.groundSpeed >= GPS_COG_MIN_GROUNDSPEED) {
        // Use GPS course over ground to correct attitude.values.yaw
        if (isFixedWing()) {
            courseOverGround = DECIDEGREES_TO_RADIANS(gpsSol.groundCourse);
            useCOG = true;
        } else {
            courseOverGround = DECIDEGREES_TO_RADIANS(gpsSol.groundCourse);

            useCOG = true;
        }

        if (useCOG && shouldInitializeGPSHeading()) {
            // Reset our reference and reinitialize quaternion.  This will likely ideally happen more than once per flight, but for now,
            // shouldInitializeGPSHeading() returns true only once.
            imuComputeQuaternionFromRPY(&qP, attitude.values.roll, attitude.values.pitch, gpsSol.groundCourse);

            useCOG = false; // Don't use the COG when we first reinitialize.  Next time around though, yes.
        }
    }
    #endif
    float gyroAverage[XYZ_AXIS_COUNT];
    gyroGetAccumulationAverage(gyroAverage);

    if (accGetAccumulationAverage(accAverage)) {
        useAcc = imuIsAccelerometerHealthy(accAverage);
    }

    imuMahonyAHRSupdate(deltaT * 1e-6f,
                        DEGREES_TO_RADIANS(gyroAverage[X]), DEGREES_TO_RADIANS(gyroAverage[Y]), DEGREES_TO_RADIANS(gyroAverage[Z]),
                        useAcc, accAverage[X], accAverage[Y], accAverage[Z],
                        useMag,
                        useCOG, courseOverGround,  imuCalcKpGain(currentTimeUs, useAcc, gyroAverage));

    imuUpdateEulerAngles();
}

#ifdef USE_YAW_SPIN_RECOVERY
void initYawSpinRecovery(int maxYawRate)
{
    bool enabledFlag;
    int threshold;

    switch (YAW_SPIN_RECOVERY_AUTO) {
    case YAW_SPIN_RECOVERY_ON:
        enabledFlag = true;
        threshold = 1950;//gyroConfig()->yaw_spin_threshold;
        break;
    case YAW_SPIN_RECOVERY_AUTO:
        enabledFlag = true;
        const int overshootAllowance = MAX(maxYawRate / 4, 200); // Allow a 25% or minimum 200dps overshoot tolerance
        threshold = constrain(maxYawRate + overshootAllowance, YAW_SPIN_RECOVERY_THRESHOLD_MIN, YAW_SPIN_RECOVERY_THRESHOLD_MAX);
        break;
    case YAW_SPIN_RECOVERY_OFF:
    default:
        enabledFlag = false;
        threshold = YAW_SPIN_RECOVERY_THRESHOLD_MAX;
        break;
    }

    yawSpinRecoveryEnabled = enabledFlag;
    yawSpinRecoveryThreshold = threshold;
}
#endif

#ifdef _USE_HW_CLI
void cliSensor(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "test") == true)
  {
    while(cliKeepLoop())
    {
  
    }
	}

    ret = true;

  if (ret != true)
  {
    cliPrintf("lcd test\n");
    cliPrintf("lcd image\n");
  }
}
#endif