#pragma once

typedef enum {
  STANDBY = 0,
  MOTOR_BURN = 1,
  COAST = 2,
  DESCENT = 3,
  LANDED = 4,
} State;

/**
 * @brief Number of states in the Unscented Kalman Filter (UKF) state vector
 */
#define UKF_STATE_DIMENSION 16

#define UKF_COVARIANCE_DIMENSION (UKF_STATE_DIMENSION - 1)

/**
 * @brief Number of measurements in the UKF measurement vector
 */
#define UKF_MEASUREMENT_DIMENSION 10

/**
 * @brief Number of flight states in the UKF state machine
 */
#define STATE_MACHINE_NUM_STATES 5

/**
 * @brief Initial values for the UKF states
 */
extern float ukf_initial_state_estimate[UKF_STATE_DIMENSION];

/**
 * @brief Initial values for the UKF covariance matrix diagonal elements
 */
extern float ukf_initial_state_covariance_diag[UKF_COVARIANCE_DIMENSION];

/**
 * @brief Diagonal elements for the state process covariance matrices of each flight state
 */
extern float ukf_state_process_covariance_diag[STATE_MACHINE_NUM_STATES][UKF_COVARIANCE_DIMENSION];

/**
 * @brief Diagonal elements for the measurement noise covariance matrices of each flight state
 */
extern float ukf_measurement_noise_covariance_diag[STATE_MACHINE_NUM_STATES]
                                                  [UKF_MEASUREMENT_DIMENSION];

#define GRAVITY_METERS_PER_SECOND_SQUARED 9.798F
#define UKF_MIN_VEL_FOR_DRAG 25.0F
#define DRAG_PARAM (-2.5e-4F)

/**
 * @brief UKF sigma point constants
 *
 * From page 21 Merwe et al. (2004)
 * doi: 10.2514/6.2004-5120
 */
#define UKF_SIGMA_SPREAD_ALPHA 3e-1F
#define UKF_SIGMA_WEIGHT_BETA 2.0F
#define UKF_SIGMA_TERTIARY_KAPPA 0.0F

#define GROUND_ALTITUDE_METERS 20.0F

#define SQRT2_INV 0.70710678118F /* 1/√2 */

/* ====================================================================
 * Three-frame sensor rotation model
 * ====================================================================
 * Sensor frame  : IC-specific axes (per datasheet)
 * Board frame   : +X forward, +Y left, +Z up
 * World frame   : inertial frame, defined by the quaternion state
 *
 * R_imu_to_board  transforms a vector from IMU sensor frame → board frame
 * R_mag_to_board  transforms a vector from mag sensor frame → board frame
 *
 * Selection at runtime via firmSettings.firmware_version:
 *   "v1.x.x" → legacy hardware  (v1 matrices)
 *   "v2.x.x" → current hardware (v2 matrices, default)
 * ==================================================================== */

/* Hardware version v2 (current) */
extern const float ukf_v2_R_imu_to_board[9]; /* 3×3 row-major */
extern const float ukf_v2_R_mag_to_board[9]; /* 3×3 row-major */

/* Hardware version v1 (legacy) */
extern const float ukf_v1_R_imu_to_board[9]; /* 3×3 row-major */
extern const float ukf_v1_R_mag_to_board[9]; /* 3×3 row-major */
