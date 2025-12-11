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
#define UKF_STATE_DIMENSION 22

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
extern double ukf_initial_state_estimate[UKF_STATE_DIMENSION];

/**
 * @brief Initial values for the UKF covariance matrix diagonal elements
 */
extern double ukf_initial_state_covariance_diag[UKF_STATE_DIMENSION - 1];

/**
 * @brief Diagonal elements for the state process covariance matrices of each flight state
 */
extern double ukf_state_process_covariance_diag[STATE_MACHINE_NUM_STATES][UKF_STATE_DIMENSION - 1];

/**
 * @brief Diagonal elements for the measurement noise covariance matrices of each flight state
 */
extern double ukf_measurement_noise_covariance_diag[STATE_MACHINE_NUM_STATES][UKF_MEASUREMENT_DIMENSION];

#define GRAVITY_METERS_PER_SECOND_SQUARED 9.798
#define UKF_MIN_VEL_FOR_DRAG 25.0
#define DRAG_PARAM (-2.5e-4)


/**
 * @brief UKF sigma point constants
 * 
 * From page 21 Merwe et al. (2004)
 * doi: 10.2514/6.2004-5120
 */
#define UKF_SIGMA_SPREAD_ALPHA 1e-2
#define UKF_SIGMA_WEIGHT_BETA 2
#define UKF_SIGMA_TERTIARY_KAPPA 0

#define GROUND_ALTITUDE_METERS 20

