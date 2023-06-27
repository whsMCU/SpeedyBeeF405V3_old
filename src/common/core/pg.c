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

#include <stddef.h>
#include <string.h>
#include <stdint.h>

#include "pg.h"

__attribute__((section(".pg_registry"))) version_info_t verstion_info =
{
  .magic_number = 0x5555AAAA,
  .version_str = "V230331R1"
};

const version_info_t *p_verstion_info = (version_info_t *)0x8000400;

__attribute__((section(".pg_registry"))) gyroConfig_t gyro_pg =
{
  .gyroCalibrationDuration = 125,        // 1.25 seconds
  .gyroMovementCalibrationThreshold = 48,
  .gyro_hardware_lpf = GYRO_HARDWARE_LPF_NORMAL,
  .gyro_lpf1_type = FILTER_PT1,
  .gyro_lpf1_static_hz = GYRO_LPF1_DYN_MIN_HZ_DEFAULT,  
      // NOTE: dynamic lpf is enabled by default so this setting is actually
      // overridden and the static lowpass 1 is disabled. We can't set this
      // value to 0 otherwise Configurator versions 10.4 and earlier will also
      // reset the lowpass filter type to PT1 overriding the desired BIQUAD setting.
  .gyro_lpf2_type = FILTER_PT1,
  .gyro_lpf2_static_hz = GYRO_LPF2_HZ_DEFAULT,
  .gyro_high_fsr = false,
  .gyro_to_use = 0,
  .gyro_soft_notch_hz_1 = 0,
  .gyro_soft_notch_cutoff_1 = 0,
  .gyro_soft_notch_hz_2 = 0,
  .gyro_soft_notch_cutoff_2 = 0,
  .checkOverflow = GYRO_OVERFLOW_CHECK_ALL_AXES,
  .gyro_offset_yaw = 0,
  .yaw_spin_recovery = YAW_SPIN_RECOVERY_AUTO,
  .yaw_spin_threshold = 1950,
  .gyro_lpf1_dyn_min_hz = GYRO_LPF1_DYN_MIN_HZ_DEFAULT,
  .gyro_lpf1_dyn_max_hz = GYRO_LPF1_DYN_MAX_HZ_DEFAULT,
  .gyro_filter_debug_axis = FD_ROLL,
  .gyro_lpf1_dyn_expo = 5,
  .simplified_gyro_filter = true,
  .simplified_gyro_filter_multiplier = SIMPLIFIED_TUNING_DEFAULT
};

const gyroConfig_t *p_gyro_pg = (gyroConfig_t *)(0x8000400+sizeof(version_info_t));


__attribute__((section(".pg_registry"))) pidProfile_t pid_pg =
{
  .pid = {
      [PID_ROLL] =  PID_ROLL_DEFAULT,
      [PID_PITCH] = PID_PITCH_DEFAULT,
      [PID_YAW] =   PID_YAW_DEFAULT,
      [PID_LEVEL] = { 50, 50, 75, 0 },
      [PID_MAG] =   { 40, 0, 0, 0 },
  },
  .pidSumLimit = PIDSUM_LIMIT,
  .pidSumLimitYaw = PIDSUM_LIMIT_YAW,
  .yaw_lowpass_hz = 100,
  .dterm_notch_hz = 0,
  .dterm_notch_cutoff = 0,
  .itermWindupPointPercent = 85,
  .pidAtMinThrottle = PID_STABILISATION_ON,
  .levelAngleLimit = 55,
  .feedforward_transition = 0,
  .yawRateAccelLimit = 0,
  .rateAccelLimit = 0,
  .itermThrottleThreshold = 250,
  .itermAcceleratorGain = 3500,
  .crash_time = 500,          // ms
  .crash_delay = 0,           // ms
  .crash_recovery_angle = 10, // degrees
  .crash_recovery_rate = 100, // degrees/second
  .crash_dthreshold = 50,     // degrees/second/second
  .crash_gthreshold = 400,    // degrees/second
  .crash_setpoint_threshold = 350, // degrees/second
  .crash_recovery = PID_CRASH_RECOVERY_OFF, // off by default
  .horizon_tilt_effect = 75,
  .horizon_tilt_expert_mode = false,
  .crash_limit_yaw = 200,
  .itermLimit = 400,
  .throttle_boost = 5,
  .throttle_boost_cutoff = 15,
  .iterm_rotation = false,
  .iterm_relax = ITERM_RELAX_RP,
  .iterm_relax_cutoff = ITERM_RELAX_CUTOFF_DEFAULT,
  .iterm_relax_type = ITERM_RELAX_SETPOINT,
  .acro_trainer_angle_limit = 20,
  .acro_trainer_lookahead_ms = 50,
  .acro_trainer_debug_axis = FD_ROLL,
  .acro_trainer_gain = 75,
  .abs_control_gain = 0,
  .abs_control_limit = 90,
  .abs_control_error_limit = 20,
  .abs_control_cutoff = 11,
  .antiGravityMode = ANTI_GRAVITY_SMOOTH,
  .dterm_lpf1_static_hz = DTERM_LPF1_DYN_MIN_HZ_DEFAULT,
      // NOTE: dynamic lpf is enabled by default so this setting is actually
      // overridden and the static lowpass 1 is disabled. We can't set this
      // value to 0 otherwise Configurator versions 10.4 and earlier will also
      // reset the lowpass filter type to PT1 overriding the desired BIQUAD setting.
  .dterm_lpf2_static_hz = DTERM_LPF2_HZ_DEFAULT,   // second Dterm LPF ON by default
  .dterm_lpf1_type = FILTER_PT1,
  .dterm_lpf2_type = FILTER_PT1,
  .dterm_lpf1_dyn_min_hz = DTERM_LPF1_DYN_MIN_HZ_DEFAULT,
  .dterm_lpf1_dyn_max_hz = DTERM_LPF1_DYN_MAX_HZ_DEFAULT,
  .launchControlMode = LAUNCH_CONTROL_MODE_NORMAL,
  .launchControlThrottlePercent = 20,
  .launchControlAngleLimit = 0,
  .launchControlGain = 40,
  .launchControlAllowTriggerReset = true,
  .use_integrated_yaw = false,
  .integrated_yaw_relax = 200,
  .thrustLinearization = 0,
  .d_min = D_MIN_DEFAULT,
  .d_min_gain = 37,
  .d_min_advance = 20,
  .motor_output_limit = 100,
  .auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY,
  .transient_throttle_limit = 0,
  .profileName = { 0 },
  .dyn_idle_min_rpm = 0,
  .dyn_idle_p_gain = 50,
  .dyn_idle_i_gain = 50,
  .dyn_idle_d_gain = 50,
  .dyn_idle_max_increase = 150,
  .feedforward_averaging = FEEDFORWARD_AVERAGING_OFF,
  .feedforward_max_rate_limit = 90,
  .feedforward_smooth_factor = 25,
  .feedforward_jitter_factor = 7,
  .feedforward_boost = 15,
  .dterm_lpf1_dyn_expo = 5,
  .level_race_mode = false,
  .vbat_sag_compensation = 0,
  .simplified_pids_mode = PID_SIMPLIFIED_TUNING_RPY,
  .simplified_master_multiplier = SIMPLIFIED_TUNING_DEFAULT,
  .simplified_roll_pitch_ratio = SIMPLIFIED_TUNING_DEFAULT,
  .simplified_i_gain = SIMPLIFIED_TUNING_DEFAULT,
  .simplified_d_gain = SIMPLIFIED_TUNING_D_DEFAULT,
  .simplified_pi_gain = SIMPLIFIED_TUNING_DEFAULT,
  .simplified_dmin_ratio = SIMPLIFIED_TUNING_D_DEFAULT,
  .simplified_feedforward_gain = SIMPLIFIED_TUNING_DEFAULT,
  .simplified_pitch_pi_gain = SIMPLIFIED_TUNING_DEFAULT,
  .simplified_dterm_filter = true,
  .simplified_dterm_filter_multiplier = SIMPLIFIED_TUNING_DEFAULT

  #ifndef USE_D_MIN
    .pid[PID_ROLL].D = 30;
    .pid[PID_PITCH].D = 32;
#endif
};

const pidProfile_t *p_pid_pg = (pidProfile_t *)(0x8000400+(sizeof(version_info_t)+sizeof(gyroConfig_t)));
