#pragma once
#include "kalman_filter_config.h"
#include "matrixhelper.h"
#include "arm_math.h"

/**
 * @brief UKF parameters
 */
typedef struct {
    double *X; // state vector
    double *P; // covariance matrix
    double *Q; // process noise matrix
    double *test;
    int flight_state; // 0 - 4 for standby - landed
    void (*state_transition_function)(const double*, double, int, double*);
} UKF;

/**
 * @brief UKF initialization
 * 
 * @param ukfh pointer to UKF parameters
 * @return 0 on success, 1 on failure
 */
int ukf_init(UKF *ukfh);

/**
 * @brief UKF predict step
 * 
 * @param ukfh pointer to UKF parameters
 * @param delta_time time difference in seconds between this predict step and last predict step
 * @return 0 on success, 1 on failure
 */
int ukf_predict(UKF *ukfh, double delta_time);

#ifdef TEST
double ukf_test_get_lambda(void);
double* ukf_test_get_Wm(void);
double* ukf_test_get_Wc(void);
double* ukf_test_get_sigmas_f(void);
void ukf_test_get_sigma_points(double sigmas[][UKF_STATE_DIMENSION]);
double* ukf_test_get_residuals(void);
double* ukf_test_get_weighted_vector_sigmas(void);
double* ukf_test_get_Q_scaled(void);
#endif