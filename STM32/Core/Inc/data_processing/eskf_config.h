#pragma once
#include <stdint.h>

/* ====================================================================
 * ESKF (Error-State Extended Kalman Filter) Configuration
 * ====================================================================
 *
 * Nominal state (6): altitude(1), velocity(1), quat(4) [w,x,y,z]
 * Error   state (5): δalt(1), δvel(1), δθ(3)
 * Measurement   (1): pressure(1) only
 * Control       (6): accel(3), gyro(3) (sensor frame, g and deg/s)
 * ==================================================================== */


/** Vector Dimensions */
#define ESKF_NOMINAL_DIM 6
#define ESKF_ERROR_DIM 5
#define ESKF_MEASUREMENT_DIM 1
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
