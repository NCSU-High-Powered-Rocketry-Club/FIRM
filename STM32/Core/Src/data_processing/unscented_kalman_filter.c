#include "unscented_kalman_filter.h"
#include "dsp/matrix_functions.h"


#define UKF_NUM_SIGMAS (2 * UKF_STATE_DIMENSION - 1)

// static array allocation
static float X[UKF_STATE_DIMENSION]; // state vector
static float P_data[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 1)]; // covariance matrix
static float Q_data[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 1)]; // process noise matrix
static float R_data[UKF_MEASUREMENT_DIMENSION * UKF_MEASUREMENT_DIMENSION]; // measurement noise matrix
static float sigmas_f_data[UKF_NUM_SIGMAS * UKF_STATE_DIMENSION]; // process sigma point matrix
static float sigmas_h_data[UKF_NUM_SIGMAS * UKF_MEASUREMENT_DIMENSION]; // measurement sigma point matrix
static float Wm[UKF_NUM_SIGMAS]; // sigma point mean weights
static float Wc[UKF_NUM_SIGMAS]; // sigma point covariance weights
static float lambda_scaling_parameter;
// temp arrays
static float temp_P_shape0_data[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 1)];
static float temp_P_shape1_data[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 1)];

static float test_data[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 1)];

static arm_matrix_instance_f32 P = {UKF_STATE_DIMENSION - 1, UKF_STATE_DIMENSION - 1, P_data};
static arm_matrix_instance_f32 Q = {UKF_STATE_DIMENSION - 1, UKF_STATE_DIMENSION - 1, Q_data};
static arm_matrix_instance_f32 R = {UKF_MEASUREMENT_DIMENSION, UKF_MEASUREMENT_DIMENSION, R_data};
static arm_matrix_instance_f32 sigmas_f = {UKF_NUM_SIGMAS, UKF_STATE_DIMENSION, sigmas_f_data};
static arm_matrix_instance_f32 sigmas_h = {UKF_NUM_SIGMAS, UKF_MEASUREMENT_DIMENSION, sigmas_h_data};

// temp matrices
static arm_matrix_instance_f32 temp_P_shape0 = {UKF_STATE_DIMENSION - 1, UKF_STATE_DIMENSION - 1, temp_P_shape0_data};
static arm_matrix_instance_f32 temp_P_shape1 = {UKF_STATE_DIMENSION - 1, UKF_STATE_DIMENSION - 1, temp_P_shape1_data};

/**
 * @brief calculate sigma point weights
 */
static void calculate_sigma_weights(void);

/**
 * @brief calculates the sigma points to be used in the state transition function
 * 
 * @param ukfh pointer to the UKF parameters handle
 * @return 0 on success, 1 on failure
 */
static int calculate_sigma_points(UKF *ukfh);


int ukf_init(UKF *ukfh) {
    // set covariance matrix to all zeros
    memset(P.pData, 0, (UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 1));
    for (int i = 0; i < UKF_STATE_DIMENSION - 1; i++) {
        // set inital values of state vector, and initial diagonals of covariance matrix
        X[i] = ukf_initial_state_estimate[i];
        P.pData[i + (UKF_STATE_DIMENSION - 1) * i] = ukf_initial_state_covariance_diag[i];
    }
    // set last element of X (because P is 21 x 21 but X is 22 x 1)
    X[UKF_STATE_DIMENSION - 1] = ukf_initial_state_estimate[UKF_STATE_DIMENSION - 1];

    ukfh->X = X;
    ukfh->P = P.pData;
    

    // DEBUG
    for (int i = 0; i < 21*21; i++) {
        Q.pData[i] = 1;
        test_data[i] = 0;
    }
    ukfh->test = test_data;
    //


    calculate_sigma_weights();
    return 0;
}

int ukf_predict(UKF *ukfh, const float delta_time) {
    // check if dt is too fast
    if (delta_time < 1e-12F) {
        return 1;
    }

    // normalize the quaternion state
    float quat_norm;
    arm_quaternion_norm_f32(&(ukfh->X[UKF_STATE_DIMENSION - 4]), &quat_norm, 1);
    for (int i = 0; i < 4; i++) {
        ukfh->X[UKF_STATE_DIMENSION - 1 - i] /= quat_norm;
    }
    calculate_sigma_points(ukfh);
}

static void calculate_sigma_weights(void) {
    lambda_scaling_parameter = UKF_SIGMA_SPREAD_ALPHA * UKF_SIGMA_SPREAD_ALPHA * (UKF_STATE_DIMENSION - 1.0F + UKF_SIGMA_TERTIARY_KAPPA) - (UKF_STATE_DIMENSION - 1.0F);
    float weight_component = 1.0F / (2.0F * (lambda_scaling_parameter + UKF_STATE_DIMENSION - 1.0F));
    for (int i = 1; i < UKF_NUM_SIGMAS; i++) {
        Wm[i] = weight_component;
        Wc[i] = weight_component;
    }
    Wm[0] = lambda_scaling_parameter / (UKF_STATE_DIMENSION - 1.0F + lambda_scaling_parameter);
    Wc[0] = lambda_scaling_parameter / (UKF_STATE_DIMENSION - 1.0F + lambda_scaling_parameter) + (1.0F - UKF_SIGMA_SPREAD_ALPHA * UKF_SIGMA_SPREAD_ALPHA + UKF_SIMGA_WEIGHT_BETA);
}

static int calculate_sigma_points(UKF *ukfh) {
    if (symmetric(&P)) {
        return 1;
    }

    arm_mat_add_f32(&P, &Q, &temp_P_shape0);
    arm_mat_scale_f32(&temp_P_shape0, lambda_scaling_parameter + (UKF_STATE_DIMENSION - 1), &temp_P_shape0);
    if (arm_mat_cholesky_f32(&temp_P_shape0, &temp_P_shape1)) {
        return 1;
    }
    ukfh->test = temp_P_shape1.pData;
    
    
    return 0;
} 


#ifdef TEST
float* ukf_test_get_Wm(void) { return Wm; }
float* ukf_test_get_Wc(void) { return Wc; }
#endif