#pragma once

/**
 * @brief Number of states in the Unscented Kalman Filter (UKF) state vector
 */
#define UKF_STATE_DIMENSION 22

/**
 * @brief Initial values for the UKF states
 */
extern double ukf_initial_state_estimate[UKF_STATE_DIMENSION];

/**
 * @brief Initial values for the UKF covariance matrix diagonal elements
 */
extern double ukf_initial_state_covariance_diag[UKF_STATE_DIMENSION - 1];

/**
 * @brief Number of measurements in the UKF measurement vector
 */
#define UKF_MEASUREMENT_DIMENSION 10

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
#define UKF_SIMGA_WEIGHT_BETA 2
#define UKF_SIGMA_TERTIARY_KAPPA 0

