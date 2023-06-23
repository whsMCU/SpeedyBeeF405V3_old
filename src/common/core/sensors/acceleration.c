/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "axis.h"
#include "filter.h"
#include "utils.h"

#include "acceleration_init.h"

#include "acceleration.h"

acc_t acc;                       // acc access functions

static void applyAccelerationTrims(const flightDynamicsTrims_t *accelerationTrims)
{
    acc.accADC[X] -= accelerationTrims->raw[X];
    acc.accADC[Y] -= accelerationTrims->raw[Y];
    acc.accADC[Z] -= accelerationTrims->raw[Z];
}

#define acc_lpf_factor 4

void accUpdate(uint32_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    static float accLPF[3];

    if(acc.dev.dataReady){
        if (!acc.dev.readFn(&acc.dev)) {
            return;
        }
        acc.dev.dataReady == false;
    }
    acc.isAccelUpdatedAtLeastOnce = true;

        imu->imuDev.isAccelUpdatedAtLeastOnce = true;
        imu->imuDev.accADC[X] = imu->imuDev.accADCRaw[X];
        imu->imuDev.accADC[Y] = imu->imuDev.accADCRaw[Y];
        imu->imuDev.accADC[Z] = imu->imuDev.accADCRaw[Z];

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        //DEBUG_SET(DEBUG_ACCELEROMETER, axis, acc.dev.ADCRaw[axis]);
        acc.accADC[axis] = acc.dev.ADCRaw[axis];
    }

    for(int axis=0;axis<3;axis++)
	{
		if (acc_lpf_factor > 0)
		{
			accLPF[axis] = accLPF[axis] * (1.0f - (1.0f / acc_lpf_factor)) + acc.accADC[axis] * (1.0f / acc_lpf_factor);
			acc.accADC[axis] = accLPF[axis];
		}
	}

    // if (accelerationRuntime.accLpfCutHz) {
    //     for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
    //         acc.accADC[axis] = biquadFilterApply(&accelerationRuntime.accFilter[axis], acc.accADC[axis]);
    //     }
    // }

    if (!accIsCalibrationComplete()) {
        performAcclerationCalibration();
    }

    applyAccelerationTrims(&sensor);

    ++sensor.accumulatedMeasurementCount;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        sensor.accumulatedMeasurements[axis] += sensor.accADC[axis];
    }




    if (featureIsEnabled(FEATURE_INFLIGHT_ACC_CAL)) {
        performInflightAccelerationCalibration(rollAndPitchTrims);
    }

    applyAccelerationTrims(accelerationRuntime.accelerationTrims);

    ++accelerationRuntime.accumulatedMeasurementCount;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        accelerationRuntime.accumulatedMeasurements[axis] += acc.accADC[axis];
    }
}

bool accGetAccumulationAverage(float *accumulationAverage)
{
    if (accelerationRuntime.accumulatedMeasurementCount > 0) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = accelerationRuntime.accumulatedMeasurements[axis] / accelerationRuntime.accumulatedMeasurementCount;
            accelerationRuntime.accumulatedMeasurements[axis] = 0.0f;
        }
        accelerationRuntime.accumulatedMeasurementCount = 0;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}
