#pragma once
#include "kalman_filter_config.h"
#include "matrixhelper.h"
#include "arm_math.h"

/**
 * @brief UKF parameters
 */
typedef struct {
    float *X; // state vector
    float *P; // covariance matrix
    float *test;
    void (*state_transition_function)(float*, float);

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
 * @param delta_time time difference float in seconds between this predict step and last predict step
 * @return 0 on success, 1 on failure
 */
int ukf_predict(UKF *ukfh, float delta_time);

#ifdef TEST
float* ukf_test_get_Wm(void);
float* ukf_test_get_Wc(void);
float* ukf_test_get_sigmas_f(void);
#endif