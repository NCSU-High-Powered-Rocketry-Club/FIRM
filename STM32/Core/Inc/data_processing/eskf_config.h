#pragma once
#include <stdint.h>

/* ====================================================================
 * ESKF (Error-State Extended Kalman Filter) Configuration
 * ====================================================================
 *
 * Nominal state (6): altitude(1), velocity(1), quat(4) [w,x,y,z]
 * Error   state (5): δalt(1), δvel(1), δθ(3)
 * Measurement   (4): pressure(1), mag_sensor(3) (normalised)
 * Control       (6): accel(3), gyro(3) (sensor frame, g and deg/s)
 * ==================================================================== */


/** Vector Dimensions */
#define ESKF_NOMINAL_DIM 6
#define ESKF_ERROR_DIM 5
#define ESKF_MEASUREMENT_DIM 4
#define ESKF_CONTROL_DIM 6

/** Nominal State Index Enum */
typedef enum {
  ESKF_POS_Z = 0,
  ESKF_VEL_Z = 1,
  ESKF_QUAT_W = 2,
  ESKF_QUAT_X = 3,
  ESKF_QUAT_Y = 4,
  ESKF_QUAT_Z = 5,
} ESKFNominalIndex;

/** Error State Index Enum */
typedef enum {
  ESKF_DPOS_Z = 0,
  ESKF_DVEL_Z = 1,
  ESKF_DTHETA_X = 2,
  ESKF_DTHETA_Y = 3,
  ESKF_DTHETA_Z = 4,
} ESKFErrorIndex;

/** Physical Constants */
#define GRAVITY_METERS_PER_SECOND_SQUARED 9.798F
#define PRESSURE_ALTITUDE_CONST 44330.0F
#define PRESSURE_EXPONENT 5.255876F

/** √2/2 constant for sensor rotation matrices */
#define SQRT2_INV 0.70710678118F

/** pressure decoupling sigmoid */
#define ESKF_PV_COUPLING_SPEED 20.0F
#define ESKF_PV_COUPLING_SHARPNESS 3.0F
#define ESKF_COAST_MIN_VELOCITY 10.0F
#define ESKF_COAST_ENTER_ACCELERATION -5.0F
#define ESKF_COAST_EXIT_ACCELERATION -2.0F
#define ESKF_COAST_CONFIRMATION_SECONDS 0.1F
#define ESKF_COAST_PRESSURE_POSITION_COUPLING 0.5F
#define ESKF_COAST_PRESSURE_COUPLING 0.04F
#define ESKF_COAST_PRESSURE_COUPLING_TIME_CONSTANT 0.5F
#define ESKF_PRESSURE_ALTITUDE_FILTER_TIME_CONSTANT 0.1F
#define ESKF_PRESSURE_DISTURBANCE_VELOCITY_ERROR 50.0F
#define ESKF_PRESSURE_RECOVERY_VELOCITY_ERROR 25.0F
#define ESKF_PRESSURE_MAX_ALTITUDE_INNOVATION 30.0F
#define ESKF_PRESSURE_RECOVERY_CONFIRMATION_SECONDS 0.1F
#define ESKF_PRESSURE_RECOVERY_RAMP_SECONDS 0.25F
#define ESKF_PRESSURE_OFFSET_FADE_TIME_CONSTANT 2.0F
#define ESKF_PRESSURE_OFFSET_FADE_MAX_ALTITUDE_ERROR 50.0F

/** Automatic flight-phase detection and standby reference tracking. */
#define ESKF_LAUNCH_ACCELERATION_G 2.0F
#define ESKF_LAUNCH_MIN_VERTICAL_ACCELERATION 15.0F
#define ESKF_LAUNCH_CONFIRMATION_SECONDS 0.15F
#define ESKF_LAUNCH_MIN_CANDIDATE_VELOCITY 3.0F
#define ESKF_LAUNCH_PRESSURE_RISE_METERS 0.5F
#define ESKF_STANDBY_PRESSURE_TIME_CONSTANT 1.0F
#define ESKF_STANDBY_IMU_TIME_CONSTANT 5.0F
#define ESKF_STANDBY_ACCELERATION_MIN_G 0.8F
#define ESKF_STANDBY_ACCELERATION_MAX_G 1.2F
#define ESKF_STANDBY_GYRO_MAX_DPS 1.0F
#define ESKF_MAGNETOMETER_MAX_DIRECTION_ERROR 0.2610524F /* 2 sin(15 deg / 2) */

/** Autonomous post-apogee stationary detector / zero-velocity update. */
#define ESKF_LANDED_ACCELERATION_MIN_G 0.85F
#define ESKF_LANDED_ACCELERATION_MAX_G 1.15F
#define ESKF_LANDED_GYRO_MAX_DPS 3.0F
#define ESKF_LANDED_MAX_PRESSURE_VELOCITY 1.5F
#define ESKF_LANDED_MAX_ESTIMATED_VELOCITY 5.0F
#define ESKF_LANDED_CONFIRMATION_SECONDS 2.0F
#define ESKF_LANDED_VELOCITY_VARIANCE 1e-4F

/** measurement and prediction noise arrays (defined in eskf_config.c) */
extern const float eskf_initial_state[ESKF_NOMINAL_DIM];
extern const float eskf_initial_cov_diag[ESKF_ERROR_DIM];
extern const float eskf_q_diag[ESKF_ERROR_DIM];
extern const float eskf_r_diag[ESKF_MEASUREMENT_DIM];

/* Sensor-to-board rotation matrices (defined in eskf_config.c)
 *                                                                     
 * Three reference frames:                                             
 *   sensor frame – each sensor IC's own coordinate axes, defined in datasheet
 *   board frame  – PCB body frame: +X forward, +Y left, +Z up (KiCad orientation to get forward)
 *   world frame  – intertial frame, quaternion state rotates from board to world
 *                                                                     
 * The following matrices handle sensor to board only. Board to world is        
 * determined by the ESKF quaternion state. 
 */

/* firmware v2 hardware (current PCB, FIRM v1.0) */
extern const float eskf_v2_R_imu_to_board[9]; /* 3x3 row-major */
extern const float eskf_v2_R_mag_to_board[9]; /* 3x3 row-major */

/* firmware v1 hardware (legacy PCB, FIRM v0.1) */
extern const float eskf_v1_R_imu_to_board[9]; /* 3x3 row-major */
extern const float eskf_v1_R_mag_to_board[9]; /* 3x3 row-major */
