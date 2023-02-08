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

float accAverage[XYZ_AXIS_COUNT];

static sensor_Dev_t sensor;

#ifdef _USE_HW_CLI
static void cliSensor(cli_args_t *args);
#endif

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

static void imuUpdateSensor(imuSensor_t *imu)
{
  if(!imu->imuDev.gyro_readFn(&sensor.imuSensor1))
  {
    return;
  }
    if(!imu->imuDev.acc_readFn(&sensor.imuSensor1))
  {
    return;
  }
  imu->imuDev.dataReady = false;

  imu->imuDev.gyroADC[X] = imu->imuDev.gyroADCRaw[X] - imu->imuDev.gyroZero[X];
  imu->imuDev.gyroADC[Y] = imu->imuDev.gyroADCRaw[Y] - imu->imuDev.gyroZero[Y];
  imu->imuDev.gyroADC[Z] = imu->imuDev.gyroADCRaw[Z] - imu->imuDev.gyroZero[Z];

  imu->imuDev.accADC[X] = imu->imuDev.accADCRaw[X];
  imu->imuDev.accADC[Y] = imu->imuDev.accADCRaw[Y];
  imu->imuDev.accADC[Z] = imu->imuDev.accADCRaw[Z];
}

void imuUpdate(void)
{
  imuUpdateSensor(&sensor.imuSensor1);
  sensor.gyroADC[X] = sensor.imuSensor1.imuDev.gyroADC[X] * sensor.imuSensor1.imuDev.scale;
  sensor.gyroADC[Y] = sensor.imuSensor1.imuDev.gyroADC[Y] * sensor.imuSensor1.imuDev.scale;
  sensor.gyroADC[Z] = sensor.imuSensor1.imuDev.gyroADC[Z] * sensor.imuSensor1.imuDev.scale;
  cliPrintf("gyro x: %.2f, y: %.2f, z: %.2f\n\r", sensor.gyroADC[X],
                                                  sensor.gyroADC[Y],
                                                  sensor.gyroADC[Z]);

  sensor.accADC[X] = sensor.imuSensor1.imuDev.accADC[X];
  sensor.accADC[Y] = sensor.imuSensor1.imuDev.accADC[Y];
  sensor.accADC[Z] = sensor.imuSensor1.imuDev.accADC[Z];
  cliPrintf("acc x: %.2f, y: %.2f, z: %.2f\n\r",  sensor.accADC[X],
                                                  sensor.accADC[Y],
                                                  sensor.accADC[Z]);
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
    if (imuRuntimeConfig.dcm_ki > 0.0f) {
        // Stop integrating if spinning beyond the certain limit
        if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
            const float dcmKiGain = imuRuntimeConfig.dcm_ki;
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

static void imuCalculateEstimatedAttitude(uint32_t currentTimeUs)
{
    static uint32_t previousIMUUpdateTime;
    bool useAcc = false;
    bool useMag = false;
    bool useCOG = false; // Whether or not correct yaw via imuMahonyAHRSupdate from our ground course
    float courseOverGround = 0; // To be used when useCOG is true.  Stored in Radians

    const int32_t deltaT = currentTimeUs - previousIMUUpdateTime;
    previousIMUUpdateTime = currentTimeUs;

    float gyroAverage[XYZ_AXIS_COUNT];
    // gyroGetAccumulationAverage(gyroAverage);

    // if (accGetAccumulationAverage(accAverage)) {
    //     useAcc = imuIsAccelerometerHealthy(accAverage);
    // }

    imuMahonyAHRSupdate(deltaT * 1e-6f,
                        DEGREES_TO_RADIANS(gyroAverage[X]), DEGREES_TO_RADIANS(gyroAverage[Y]), DEGREES_TO_RADIANS(gyroAverage[Z]),
                        useAcc, accAverage[X], accAverage[Y], accAverage[Z],
                        useMag,
                        useCOG, courseOverGround,  imuCalcKpGain(currentTimeUs, useAcc, gyroAverage));

    imuUpdateEulerAngles();
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