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
    // force P to be symmetric
    if (symmetric(&P)) {
        return 1;
    }

    // calculate cholesky square root with added noise
    arm_mat_add_f32(&P, &Q, &temp_P_shape0);
    arm_mat_scale_f32(&temp_P_shape0, lambda_scaling_parameter + (UKF_STATE_DIMENSION - 1), &temp_P_shape0);
    if (arm_mat_cholesky_f32(&temp_P_shape0, &temp_P_shape1)) {
        return 1; // matrix not positive definite
    }
    
    // first sigma point is just the state, X
    memcpy(&sigmas_f_data[0], X, UKF_STATE_DIMENSION * sizeof(float));

    // build sigma points
    for (int i = 0; i < UKF_STATE_DIMENSION - 1; i++) {
        
        // get indices in sigma_f data for the (i'th + 1) row and (i'th + 22'nd) row
        float *sigma_plus = &sigmas_f_data[(i + 1) * UKF_STATE_DIMENSION];
        float *sigma_minus = &sigmas_f_data[(i + UKF_STATE_DIMENSION) * UKF_STATE_DIMENSION];

        // copy X into both sigmas
        memcpy(sigma_plus, X, UKF_STATE_DIMENSION * sizeof(float));
        memcpy(sigma_minus, X, UKF_STATE_DIMENSION * sizeof(float));

        // add or subtract the cholesky column to the vector part of the sigma rows
        for (int k = 0; k < UKF_STATE_DIMENSION - 5; k++) {
            float cholesky_value = temp_P_shape1.pData[k*(UKF_STATE_DIMENSION - 1) + i];
            sigma_plus[k] += cholesky_value;
            sigma_minus[k] -= cholesky_value;
        }
        
        // extract the rotation vector component of the cholesky columns for this loop
        float chol_rotvec[3] = {
            temp_P_shape1.pData[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 4) + i],
            temp_P_shape1.pData[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 3) + i],
            temp_P_shape1.pData[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 2) + i],
        };
        float quat_chol[4];
        // convert the rotation vector component to a quaternion
        rotvec_to_quat(chol_rotvec, quat_chol);
        
        // get quaternion parts of X and sigma points
        float *quat_X = &X[UKF_STATE_DIMENSION - 4];
        float *quat_sigma_plus = &sigma_plus[UKF_STATE_DIMENSION - 4];
        float *quat_sigma_minus = &sigma_minus[UKF_STATE_DIMENSION - 4];

        // quat_plus = quat_X * quat_chol
        arm_quaternion_product_single_f32(quat_X, quat_chol, quat_sigma_plus);
        
        // quat_minus = quat_X * conj(quat_chol)
        arm_quaternion_conjugate_f32(quat_chol, quat_chol, 1);
        arm_quaternion_product_single_f32(quat_X, quat_chol, quat_sigma_minus);
    }
    
    
    return 0;
} 


#ifdef TEST
float* ukf_test_get_Wm(void) { return Wm; }
float* ukf_test_get_Wc(void) { return Wc; }
float* ukf_test_get_sigmas_f(void) { return sigmas_f_data; }
#endif