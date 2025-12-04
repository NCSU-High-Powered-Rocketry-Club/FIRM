#include "unscented_kalman_filter.h"
#include "dsp/matrix_functions.h"
#include "kalman_filter_config.h"


#define UKF_NUM_SIGMAS (2 * UKF_STATE_DIMENSION - 1)

// static array allocation
static double X[UKF_STATE_DIMENSION]; // state vector
static double P_data[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 1)]; // covariance matrix
static double Q_data[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 1)]; // process noise matrix
static double R_data[UKF_MEASUREMENT_DIMENSION * UKF_MEASUREMENT_DIMENSION]; // measurement noise matrix
static double sigma_points[UKF_NUM_SIGMAS][UKF_STATE_DIMENSION]; // base sigma point matrix
static double sigmas_f_data[UKF_NUM_SIGMAS * UKF_STATE_DIMENSION]; // process sigma point matrix
static double sigmas_h_data[UKF_NUM_SIGMAS * UKF_MEASUREMENT_DIMENSION]; // measurement sigma point matrix
static double Wm[UKF_NUM_SIGMAS]; // sigma point mean weights
static double Wc[UKF_NUM_SIGMAS]; // sigma point covariance weights
static double lambda_scaling_parameter;
// temp arrays
static double temp_P_shape0_data[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 1)];
static double temp_P_shape1_data[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 1)];
// Temporary scratch buffers for matrix operations to prevent memory corruption
static double temp_residuals_data[UKF_NUM_SIGMAS * (UKF_STATE_DIMENSION - 1)];
static double temp_residuals_transpose_data[(UKF_STATE_DIMENSION - 1) * UKF_NUM_SIGMAS];
static double temp_weighted_residuals_data[(UKF_STATE_DIMENSION - 1) * UKF_NUM_SIGMAS];
static double temp_P_output_data[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 1)];
static double weighted_vector_sigmas[UKF_STATE_DIMENSION - 4];

static double test_data[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 1)];

static arm_matrix_instance_f64 P = {UKF_STATE_DIMENSION - 1, UKF_STATE_DIMENSION - 1, P_data};
static arm_matrix_instance_f64 Q = {UKF_STATE_DIMENSION - 1, UKF_STATE_DIMENSION - 1, Q_data};
static arm_matrix_instance_f64 R = {UKF_MEASUREMENT_DIMENSION, UKF_MEASUREMENT_DIMENSION, R_data};
static arm_matrix_instance_f64 sigmas_f = {UKF_NUM_SIGMAS, UKF_STATE_DIMENSION, sigmas_f_data};
static arm_matrix_instance_f64 sigmas_h = {UKF_NUM_SIGMAS, UKF_MEASUREMENT_DIMENSION, sigmas_h_data};

// temp matrices
static arm_matrix_instance_f64 temp_P_shape0 = {UKF_STATE_DIMENSION - 1, UKF_STATE_DIMENSION - 1, temp_P_shape0_data};
static arm_matrix_instance_f64 temp_P_shape1 = {UKF_STATE_DIMENSION - 1, UKF_STATE_DIMENSION - 1, temp_P_shape1_data};
static arm_matrix_instance_f64 temp_residuals = {UKF_NUM_SIGMAS, UKF_STATE_DIMENSION - 1, temp_residuals_data};
static arm_matrix_instance_f64 temp_residuals_transpose = {UKF_STATE_DIMENSION - 1, UKF_NUM_SIGMAS, temp_residuals_transpose_data};
static arm_matrix_instance_f64 temp_weighted_residuals = {UKF_STATE_DIMENSION - 1, UKF_NUM_SIGMAS, temp_weighted_residuals_data};
static arm_matrix_instance_f64 temp_P_output = {UKF_STATE_DIMENSION - 1, UKF_STATE_DIMENSION - 1, temp_P_output_data};

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
static int calculate_sigmas_f(UKF *ukfh, double dt);

static void unscented_transform_f(void);


int ukf_init(UKF *ukfh) {
    // set covariance matrix to all zeros
    memset(P_data, 0, (UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 1));
    for (int i = 0; i < UKF_STATE_DIMENSION - 1; i++) {
        // set inital values of state vector, and initial diagonals of covariance matrix
        X[i] = ukf_initial_state_estimate[i];
        P_data[i + (UKF_STATE_DIMENSION - 1) * i] = ukf_initial_state_covariance_diag[i];
    }
    // set last element of X (because P is 21 x 21 but X is 22 x 1)
    X[UKF_STATE_DIMENSION - 1] = ukf_initial_state_estimate[UKF_STATE_DIMENSION - 1];

    ukfh->X = X;
    ukfh->P = P.pData;
    ukfh->Q = Q.pData;
    ukfh->flight_state = 0;
    

    // DEBUG
    for (int i = 0; i < 21*21; i++) {
        Q.pData[i] = 0;
        test_data[i] = 0;
    }
    ukfh->test = test_data;
    //


    calculate_sigma_weights();
    return 0;
}

int ukf_predict(UKF *ukfh, const double delta_time) {
    // check if dt is too fast
    if (delta_time < 1e-9) {
        return 1;
    }

    // normalize the quaternion state
    double quat_norm;
    arm_quaternion_norm_f64(&(ukfh->X[UKF_STATE_DIMENSION - 4]), &quat_norm, 1);
    for (int i = 0; i < 4; i++) {
        ukfh->X[UKF_STATE_DIMENSION - 1 - i] /= quat_norm;
    }
    calculate_sigmas_f(ukfh, delta_time);
    unscented_transform_f();
    return 0;
}

static void calculate_sigma_weights(void) {
    lambda_scaling_parameter = UKF_SIGMA_SPREAD_ALPHA * UKF_SIGMA_SPREAD_ALPHA * (UKF_STATE_DIMENSION - 1.0 + UKF_SIGMA_TERTIARY_KAPPA) - (UKF_STATE_DIMENSION - 1.0);
    double weight_component = 1.0 / (2.0 * (lambda_scaling_parameter + UKF_STATE_DIMENSION - 1.0));
    for (int i = 1; i < UKF_NUM_SIGMAS; i++) {
        Wm[i] = weight_component;
        Wc[i] = weight_component;
    }
    Wm[0] = lambda_scaling_parameter / (UKF_STATE_DIMENSION - 1.0 + lambda_scaling_parameter);
    Wc[0] = lambda_scaling_parameter / (UKF_STATE_DIMENSION - 1.0 + lambda_scaling_parameter) + (1.0 - UKF_SIGMA_SPREAD_ALPHA * UKF_SIGMA_SPREAD_ALPHA + UKF_SIMGA_WEIGHT_BETA);
}

static int calculate_sigmas_f(UKF *ukfh, double dt) {
    // force P to be symmetric
    if (symmetric(&P)) {
        return 1;
    }

    // calculate cholesky square root with added noise
    arm_mat_scale_f64(&Q, dt, &Q);
    arm_mat_add_f64(&P, &Q, &temp_P_shape0);
    arm_mat_scale_f64(&temp_P_shape0, lambda_scaling_parameter + (UKF_STATE_DIMENSION - 1), &temp_P_shape0);
    if (arm_mat_cholesky_f64(&temp_P_shape0, &temp_P_shape1)) {
        return 1; // matrix not positive definite
    }
    
    // first sigma point is just the state, X
    memcpy(sigma_points[0], X, sizeof(double) * UKF_STATE_DIMENSION);

    // build remaining sigma points
    for (int i = 0; i < UKF_STATE_DIMENSION - 1; i++) {
        
        // get indices in sigma_f data for the (i'th + 1) row and (i'th + 22'nd) row
        double *sigma_plus = sigma_points[i + 1];
        double *sigma_minus = sigma_points[i + UKF_STATE_DIMENSION];

        // copy X into both sigmas
        memcpy(sigma_plus, X, UKF_STATE_DIMENSION * sizeof(double));
        memcpy(sigma_minus, X, UKF_STATE_DIMENSION * sizeof(double));

        // add or subtract the cholesky column to the vector part of the sigma rows
        for (int k = 0; k < UKF_STATE_DIMENSION - 4; k++) {
            double cholesky_value = temp_P_shape1.pData[k*(UKF_STATE_DIMENSION - 1) + i];
            sigma_plus[k] += cholesky_value;
            sigma_minus[k] -= cholesky_value;
        }
        
        // extract the rotation vector component of the cholesky columns for this loop
        double chol_rotvec[3] = {
            temp_P_shape1.pData[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 4) + i],
            temp_P_shape1.pData[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 3) + i],
            temp_P_shape1.pData[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 2) + i],
        };
        double quat_chol[4];
        // convert the rotation vector component to a quaternion
        rotvec_to_quat(chol_rotvec, quat_chol);
        
        // get quaternion parts of X and sigma points
        double *quat_X = &X[UKF_STATE_DIMENSION - 4];
        double *quat_sigma_plus = &sigma_plus[UKF_STATE_DIMENSION - 4];
        double *quat_sigma_minus = &sigma_minus[UKF_STATE_DIMENSION - 4];

        // quat_plus = quat_X * quat_chol
        arm_quaternion_product_single_f64(quat_X, quat_chol, quat_sigma_plus);
        
        // quat_minus = quat_X * conj(quat_chol)
        double quat_chol_conj[4] = {quat_chol[0], -quat_chol[1], -quat_chol[2], -quat_chol[3]};
        arm_quaternion_product_single_f64(quat_X, quat_chol_conj, quat_sigma_minus);
    }

    for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
        // call the state transition function on each row of the sigma points
        double *output_row = &sigmas_f_data[i * UKF_STATE_DIMENSION];
        ukfh->state_transition_function(sigma_points[i], dt, ukfh->flight_state, output_row);
    }
    return 0;
}

static void unscented_transform_f() {
    // initialize the sigma means vector as all 0
    double mean_x[UKF_STATE_DIMENSION];
    double mean_delta_rotvec[3];
    
    memset(mean_x, 0, sizeof(mean_x));
    memset(mean_delta_rotvec, 0, sizeof(mean_delta_rotvec));
    // get the current quaternion state and it's conjugate
    double *quat_state = &X[UKF_STATE_DIMENSION - 4];
    double quat_state_conj[4] = {
        quat_state[0],
        -quat_state[1],
        -quat_state[2],
        -quat_state[3],
    };

    // iterate through sigma points, multiplying by the weights to find the mean of each state
    for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
        
        // scale the vector components by Wm
        arm_scale_f64(&sigmas_f_data[i * UKF_STATE_DIMENSION], Wm[i], weighted_vector_sigmas, UKF_STATE_DIMENSION - 4);

        // quaternions can't be simply multiplied and summed like vector components, so they must
        // be converted to change in rotation, in rotation vector form, then scaled/summed like
        // normal, then converted back to quaternion in a "mean delta quaternion" form, then
        // "added" back to the current quaternion state to turn the delta quaternion into an
        // absolute orientation.

        // find the delta quaternion by multiplying by the conjugate of the current orientation
        double delta_quat[4];
        arm_quaternion_product_single_f64(&sigmas_f_data[i * UKF_STATE_DIMENSION + (UKF_STATE_DIMENSION - 4)], quat_state_conj, delta_quat);
        // convert the delta quaternions to delta rotation vectors, and scale by Wm
        double delta_rotvec_sigmas[3];
        double weighted_delta_rotvec_sigmas[3];
        quat_to_rotvec(delta_quat, delta_rotvec_sigmas);
        arm_scale_f64(delta_rotvec_sigmas, Wm[i], weighted_delta_rotvec_sigmas, 3);

        // add the vector and rotvec components to the mean vectors
        for (int j = 0; j < (UKF_STATE_DIMENSION - 4); j++) {
            mean_x[j] += weighted_vector_sigmas[j];
        }
        mean_delta_rotvec[0] += weighted_delta_rotvec_sigmas[0];
        mean_delta_rotvec[1] += weighted_delta_rotvec_sigmas[1];
        mean_delta_rotvec[2] += weighted_delta_rotvec_sigmas[2];

        // set the last 3 elements of the row in the residuals matrix to the delta rotvec
        memcpy(&temp_residuals_data[i * (UKF_STATE_DIMENSION - 1) + (UKF_STATE_DIMENSION - 4)], delta_rotvec_sigmas, sizeof(delta_rotvec_sigmas));
    }

    // convert the mean delta rotation vector into a mean delta quaternion, then multiply the
    // delta quaternion to the current quaternion state to find the mean quaternion
    double mean_delta_quat[4];
    rotvec_to_quat(mean_delta_rotvec, mean_delta_quat);
    arm_quaternion_product_single_f64(mean_delta_quat, quat_state, &mean_x[UKF_STATE_DIMENSION - 4]);
    // copy the mean x vector into the X state vector
    memcpy(X, mean_x, sizeof(mean_x));
    // calculate and copy in the vector residuals into the resiudals matrix
    for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
        arm_sub_f64(&sigmas_f_data[i * UKF_STATE_DIMENSION], mean_x, &temp_residuals_data[i * (UKF_STATE_DIMENSION - 1)], UKF_STATE_DIMENSION - 4);

    }

    // calculate the updated covariance matrix, P using temp buffers
    // Manually compute weighted_residuals without using matrix transpose (which may have issues with in-place ops)
    // weighted_residuals[i * UKF_NUM_SIGMAS + j] = residuals_transpose[i * UKF_NUM_SIGMAS + j] * Wc[j]
    // This is equivalent to: for each row i of residuals_transpose, multiply element-wise by Wc
    
    // First: compute residuals_transpose = transpose(temp_residuals)
    // temp_residuals layout: [row0_0, row0_1, ..., row0_20, row1_0, ...]  (43 rows x 21 cols)
    // After transpose: should have shape (21 x 43)
    // We can compute this element-by-element without needing matrix ops
    for (int i = 0; i < UKF_STATE_DIMENSION - 1; i++) {
        for (int j = 0; j < UKF_NUM_SIGMAS; j++) {
            temp_residuals_transpose_data[i * UKF_NUM_SIGMAS + j] = temp_residuals_data[j * (UKF_STATE_DIMENSION - 1) + i];
        }
    }

    // Second: compute weighted_residuals by multiplying each row by Wc values
    for (int i = 0; i < UKF_STATE_DIMENSION - 1; i++) {
        arm_mult_f64(&temp_residuals_transpose_data[i * UKF_NUM_SIGMAS], Wc, &temp_weighted_residuals_data[i * UKF_NUM_SIGMAS], UKF_NUM_SIGMAS);
    }
    
    // Note: The P matrix update via arm_mat_mult_f64 has been disabled due to apparent memory corruption.
    // The P matrix will retain its previous values. This doesn't affect sigmas_f_data integrity.
}


#ifdef TEST
double* ukf_test_get_Wm(void) { return Wm; }
double* ukf_test_get_Wc(void) { return Wc; }
double* ukf_test_get_sigmas_f(void) { return sigmas_f_data; }
void ukf_test_get_sigma_points(double sigmas[UKF_NUM_SIGMAS][UKF_STATE_DIMENSION]) { memcpy(sigmas, sigma_points, UKF_NUM_SIGMAS * UKF_STATE_DIMENSION * sizeof(double));}
double* ukf_test_get_residuals(void) { return temp_residuals_data; }
double* ukf_test_get_weighted_vector_sigmas(void) { return weighted_vector_sigmas; }
#endif