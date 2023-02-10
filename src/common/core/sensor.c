/*
 * sensor.c
 *
 *  Created on: 2020. 12. 20.
 *      Author: WANG
 */

#include "sensor.h"
#include "bmi270.h"
#include "cli.h"
#include "led.h"

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

static sensor_Dev_t sensor;

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

static void imuComputeRotationMatrix(void){
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

static void imuUpdateSensor(imuSensor_t *imu)
{
    bool gyroReady, accReady;

    gyroReady = imu->imuDev.InterruptStatus & 0x40;
    accReady = imu->imuDev.InterruptStatus & 0x80;

    if(gyroReady)
    {
        imu->imuDev.gyro_readFn(&sensor.imuSensor1);
        imu->imuDev.gyroADC[X] = imu->imuDev.gyroADCRaw[X] - imu->imuDev.gyroZero[X];
        imu->imuDev.gyroADC[Y] = imu->imuDev.gyroADCRaw[Y] - imu->imuDev.gyroZero[Y];
        imu->imuDev.gyroADC[Z] = imu->imuDev.gyroADCRaw[Z] - imu->imuDev.gyroZero[Z];
    }

    if(accReady)
    {
        imu->imuDev.acc_readFn(&sensor.imuSensor1);
        imu->imuDev.accADC[X] = imu->imuDev.accADCRaw[X];
        imu->imuDev.accADC[Y] = imu->imuDev.accADCRaw[Y];
        imu->imuDev.accADC[Z] = imu->imuDev.accADCRaw[Z];

    }
    imu->imuDev.dataReady = false;
}

void imuUpdate(void)
{
    static float accLPF[3];
    static float gyroLPF[3];

    imuUpdateSensor(&sensor.imuSensor1);
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
    {
        sensor.gyroADC[axis] = sensor.imuSensor1.imuDev.gyroADC[axis] * sensor.imuSensor1.imuDev.scale;
    }
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
    {
        // integrate using trapezium rule to avoid bias
        // gyro_accumulatedMeasurements[axis] += 0.5f * (gyroPrevious[axis] + sensor.gyroADC[axis]) * sensor.imuSensor1.imuDev.targetLooptime;
        // gyroPrevious[axis] = sensor.gyroADC[axis];
        gyroLPF[axis] = gyroLPF[axis] * (1.0f - (1.0f / acc_lpf_factor)) + sensor.gyroADC[axis] * (1.0f / acc_lpf_factor);
        gyro_accumulatedMeasurements[axis] = gyroLPF[axis];
    }
    gyro_accumulatedMeasurementCount++;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
    {
        sensor.accADC[axis] = sensor.imuSensor1.imuDev.accADC[axis];
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
    imuCalculateEstimatedAttitude(micros());
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
        const uint32_t accumulatedMeasurementTimeUs = gyro_accumulatedMeasurementCount * 1;//gyro.targetLooptime;
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

void DEBUG_print(void)
{
    cliPrintf("IMU R: %d, P: %d, Y: %d\n\r",    attitude.values.roll,
                                                attitude.values.pitch,
                                                attitude.values.yaw);

    // cliPrintf("gyro x: %.2f, y: %.2f, z: %.2f\n\r", sensor.gyroADC[X],
    //                                                 sensor.gyroADC[Y],
    //                                                 sensor.gyroADC[Z]);
}

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