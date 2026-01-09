#pragma once
#include "kalman_filter_config.h"
#include "matrixhelper.h"
#include "state_machine.h"
#include "firm_tasks.h"

#define SQRT2_F 1.414213562373095F
#define PI_F 3.141592653589793F

/**
 * @brief UKF parameters
 */
typedef struct UKF {
    float *X; // state vector
    float *P; // covariance matrix
    float *Q; // process noise matrix
    float *R; // measurement nosie matrix
    State* flight_state;
    void (*state_transition_function)(const float*, float, State, float*);
    void (*measurement_function)(const float*, const struct UKF*, float*);
    float *measurement_errors; // associated errors for each measurement
    float *measurement_vector; // the raw measurements given to the update function

    float initial_pressure;
    float mag_world[3];
} UKF;

/**
 * @brief UKF initialization
 * 
 * @param ukfh pointer to UKF parameters
 * @param initial_acceleration the initial x, y, z acceleration on filter startup
 * @param initial_magnetic_field the initial x, y, z magnetic field on filter startup
 * @return 0 on success, 1 on failure
 */
int ukf_init(UKF *ukfh, float initial_pressure, float* initial_acceleration, float* initial_magnetic_field);

/**
 * @brief UKF predict step
 * 
 * @param ukfh pointer to UKF parameters
 * @param delta_time time difference in seconds between this predict step and last predict step
 * @return 0 on success, 1 on failure
 */
int ukf_predict(UKF *ukfh, float delta_time);

int ukf_update(UKF *ukfh, float *measurement);

void calculate_initial_orientation(const float *imu_accel, const float *mag_field, float *init_quaternion, float *mag_world_frame);

#ifdef TEST
float ukf_test_get_lambda(void);
float* ukf_test_get_Wm(void);
float* ukf_test_get_Wc(void);
float* ukf_test_get_sigmas_f(void);
float* ukf_test_get_sigmas_h(void);
void ukf_test_get_sigma_points(float sigmas[][UKF_STATE_DIMENSION]);
float* ukf_test_get_residuals(void);
float* ukf_test_get_weighted_vector_sigmas(void);
float* ukf_test_get_Q_scaled(void);
float* ukf_test_get_S(void);
float* ukf_test_get_pred_z(void);
#endif