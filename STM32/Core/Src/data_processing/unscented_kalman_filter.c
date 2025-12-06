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
static double sigmas_f[UKF_NUM_SIGMAS * UKF_STATE_DIMENSION]; // process sigma point matrix
static double sigmas_h[UKF_NUM_SIGMAS * UKF_MEASUREMENT_DIMENSION]; // measurement sigma point matrix
static double cholesky_data[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 1)];
static double Wm[UKF_NUM_SIGMAS]; // sigma point mean weights
static double Wc[UKF_NUM_SIGMAS]; // sigma point covariance weights
static double lambda_scaling_parameter;
static double predicted_measurement[UKF_MEASUREMENT_DIMENSION];
static double innovation_data[UKF_MEASUREMENT_DIMENSION * UKF_MEASUREMENT_DIMENSION];
static double innovation_inverse_data[UKF_MEASUREMENT_DIMENSION * UKF_MEASUREMENT_DIMENSION];
static double P_cross_covariance_data[(UKF_STATE_DIMENSION - 1) * UKF_MEASUREMENT_DIMENSION];
static double kalman_gain_data[(UKF_STATE_DIMENSION - 1) * UKF_MEASUREMENT_DIMENSION];
static double measurement_errors[UKF_MEASUREMENT_DIMENSION];

static arm_matrix_instance_f64 P = {UKF_STATE_DIMENSION - 1, UKF_STATE_DIMENSION - 1, P_data};
static arm_matrix_instance_f64 Q = {UKF_STATE_DIMENSION - 1, UKF_STATE_DIMENSION - 1, Q_data};
static arm_matrix_instance_f64 R = {UKF_MEASUREMENT_DIMENSION, UKF_MEASUREMENT_DIMENSION, R_data};
static arm_matrix_instance_f64 cholesky = {UKF_STATE_DIMENSION - 1, UKF_STATE_DIMENSION - 1, cholesky_data};
static arm_matrix_instance_f64 innovation_covariance = {UKF_MEASUREMENT_DIMENSION, UKF_MEASUREMENT_DIMENSION, innovation_data};
static arm_matrix_instance_f64 innovation_covariance_inverse = {UKF_MEASUREMENT_DIMENSION, UKF_MEASUREMENT_DIMENSION, innovation_inverse_data};
static arm_matrix_instance_f64 P_cross_covariance = {(UKF_STATE_DIMENSION - 1), UKF_MEASUREMENT_DIMENSION, P_cross_covariance_data};
static arm_matrix_instance_f64 kalman_gain = {(UKF_STATE_DIMENSION - 1), UKF_MEASUREMENT_DIMENSION, kalman_gain_data};

// temp arrays
static double temp_residuals_data[UKF_NUM_SIGMAS * (UKF_STATE_DIMENSION - 1)];
static double temp_residuals_transpose_data[(UKF_STATE_DIMENSION - 1) * UKF_NUM_SIGMAS];
static double temp_weighted_residuals_transpose_data[(UKF_STATE_DIMENSION - 1) * UKF_NUM_SIGMAS];
static double weighted_vector_sigmas[UKF_STATE_DIMENSION - 4];
static double Q_scaled_data[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 1)];

// temp matrices
static arm_matrix_instance_f64 temp_residuals = {UKF_NUM_SIGMAS, UKF_STATE_DIMENSION - 1, temp_residuals_data};
static arm_matrix_instance_f64 temp_residuals_transpose = {UKF_STATE_DIMENSION - 1, UKF_NUM_SIGMAS, temp_residuals_transpose_data};
static arm_matrix_instance_f64 temp_weighted_residuals_transpose = {UKF_STATE_DIMENSION - 1, UKF_NUM_SIGMAS, temp_weighted_residuals_transpose_data};
static arm_matrix_instance_f64 Q_scaled = {UKF_STATE_DIMENSION - 1, UKF_STATE_DIMENSION - 1, Q_scaled_data};

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

static int unscented_transform_f(void);

static int unscented_transform_h(double measurement_mean[UKF_MEASUREMENT_DIMENSION], arm_matrix_instance_f64 *innovation_cov);

static int calculate_cross_covariance(const double *measurement_mean, arm_matrix_instance_f64 *P_cross_covariance);


int ukf_init(UKF *ukfh) {
    // set covariance, process noise, and measurement noise matrix to all zeros
    memset(P_data, 0, sizeof(P_data));
    memset(Q_data, 0, sizeof(Q_data));
    memset(R_data, 0, sizeof(R_data));
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
    ukfh->R = R.pData;
    ukfh->flight_state = 0;
    ukfh->measurement_errors = measurement_errors;


    calculate_sigma_weights();
    return 0;
}

int ukf_predict(UKF *ukfh, const double delta_time) {
    // check if dt is too fast
    if (delta_time < 1e-9) {
        return 1;
    }

    // normalize the quaternion state
    quaternion_normalize_f64(&(ukfh->X[UKF_STATE_DIMENSION - 4]));
    int status = calculate_sigmas_f(ukfh, delta_time);
    if (status == 1) {
        // general error
        return 1;
    }
    if (status == 2) {
        // cholesky
        return 2;
    }
    status = unscented_transform_f();
    if (status) {
        // transform matrix multiply for P
        return 3;
    }
    return 0;
}

int ukf_update(UKF *ukfh, double *measurement) {
    // pass sigmas through the measurement function to determine how the predicted state would
    // be reflected in sensor measurements
    double measurement_sigmas[UKF_MEASUREMENT_DIMENSION];
    for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
        ukfh->measurement_function(&sigmas_f[i * UKF_STATE_DIMENSION], ukfh, measurement_sigmas);
        memcpy(&sigmas_h[i * UKF_MEASUREMENT_DIMENSION], measurement_sigmas, sizeof(measurement_sigmas));
    }

    // perform unscented transform to find mean of measurement sigma points, and the covariance of
    // the measurments
    if (unscented_transform_h(predicted_measurement, &innovation_covariance))
        return 1;
    
    // the arm_mat_inverse_f64 function says that the input matrix is const, but it actually
    // modifies it, so we are temporarily storing innovation data somewhere else and replacing it
    // after
    memcpy(kalman_gain_data, innovation_data, sizeof(innovation_data));
    if (arm_mat_inverse_f64(&innovation_covariance, &innovation_covariance_inverse)) 
        return 2;
    memcpy(innovation_data, kalman_gain_data, sizeof(innovation_data));

    // compute cross covariance between sigma points in state space (sigmas_f) and in measurement space (sigmas_h)
    if (calculate_cross_covariance(predicted_measurement, &P_cross_covariance)) {
        return 3;
    }
    
    // calculate the kalman gain, which defines how much to weigh the prediction by compared
    // to the sensor measurements
    if (arm_mat_mult_f64(&P_cross_covariance, &innovation_covariance_inverse, &kalman_gain)) {
        return 4;
    }

    // calculate measurement error metric for debug and state change purposes. These values can
    // be used to determine when a measurement is outside of the expected range and can help tune
    // the filter, remove outliers, or detecting state changes
    double measurement_residuals[UKF_MEASUREMENT_DIMENSION];
    arm_sub_f64(measurement, predicted_measurement, measurement_residuals, UKF_MEASUREMENT_DIMENSION);
    mat_vec_mult_f64(&innovation_covariance_inverse, measurement_residuals, ukfh->measurement_errors);
    arm_mult_f64(measurement_residuals, ukfh->measurement_errors, ukfh->measurement_errors, UKF_MEASUREMENT_DIMENSION);
    
    // calculate the change in the state vector based on kalman gain
    double delta_x[UKF_STATE_DIMENSION - 1];
    mat_vec_mult_f64(&kalman_gain, measurement_residuals, delta_x);
    // if we are not in standby state, do not change the accelerometer/gyroscope offsets at all
    if (ukfh->flight_state != 0) {
        memset(&delta_x[12], 0, 6 * sizeof(double));
    }
    // extract rotation vector component of delta_x and change to delta quaternion
    double delta_quat[4];
    rotvec_to_quat(&delta_x[UKF_STATE_DIMENSION - 4], delta_quat);
    // update state vector by multiplying the delta quaternion and adding the delta x vector components
    double next_quat[4];
    quaternion_product_f64(delta_quat, &X[UKF_STATE_DIMENSION - 4], next_quat);
    memcpy(&X[UKF_STATE_DIMENSION - 4], next_quat, sizeof(next_quat));
    for (int i = 0; i < UKF_STATE_DIMENSION - 4; i++) {
        X[i] += delta_x[i];
    }
    
    // compute next covariance matrix, P
    // new_P = P - (kalman_gain * innovation_covariance * kalman_gain^T)
    double kalman_gain_transpose_data[UKF_MEASUREMENT_DIMENSION * (UKF_STATE_DIMENSION - 1)];
    arm_matrix_instance_f64 kalman_gain_transpose = {UKF_MEASUREMENT_DIMENSION, UKF_STATE_DIMENSION - 1, kalman_gain_transpose_data};
    if (arm_mat_trans_f64(&kalman_gain, &kalman_gain_transpose))
        return 5;
    double temp_matrix_data[UKF_MEASUREMENT_DIMENSION * (UKF_STATE_DIMENSION - 1)];
    arm_matrix_instance_f64 temp_matrix = {UKF_MEASUREMENT_DIMENSION, UKF_STATE_DIMENSION - 1, temp_matrix_data};
    if (arm_mat_mult_f64(&innovation_covariance, &kalman_gain_transpose, &temp_matrix))
        return 6;
    arm_matrix_instance_f64 *negative_delta_P = &Q_scaled; // reusing this matrix
    if (negative_delta_P->numCols != (UKF_STATE_DIMENSION - 1) || negative_delta_P->numRows != (UKF_STATE_DIMENSION - 1))
        return 7;
    if (arm_mat_mult_f64(&kalman_gain, &temp_matrix, negative_delta_P))
        return 8;
    if (arm_mat_sub_f64(&P, negative_delta_P, &P))
        return 9;
    
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
    Wc[0] = lambda_scaling_parameter / (UKF_STATE_DIMENSION - 1.0 + lambda_scaling_parameter) + (1.0 - UKF_SIGMA_SPREAD_ALPHA * UKF_SIGMA_SPREAD_ALPHA + UKF_SIGMA_WEIGHT_BETA);
}

static int calculate_sigmas_f(UKF *ukfh, double dt) {
    // force P to be symmetric
    if (symmetrize(&P)) {
        return 1;
    }

    // calculate cholesky square root with added noise
    mat_scale_f64(&Q, dt, &Q_scaled);
    mat_add_f64(&P, &Q_scaled, &Q_scaled);
    mat_scale_f64(&Q_scaled, lambda_scaling_parameter + (UKF_STATE_DIMENSION - 1), &Q_scaled);
    arm_status status = arm_mat_cholesky_f64(&Q_scaled, &cholesky);
    if (status == ARM_MATH_DECOMPOSITION_FAILURE) {
        return 2; // matrix not positive definite
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
            double cholesky_value = cholesky_data[k*(UKF_STATE_DIMENSION - 1) + i];
            sigma_plus[k] += cholesky_value;
            sigma_minus[k] -= cholesky_value;
        }
        
        // extract the rotation vector component of the cholesky columns for this loop
        double chol_rotvec[3] = {
            cholesky_data[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 4) + i],
            cholesky_data[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 3) + i],
            cholesky_data[(UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 2) + i],
        };
        double quat_chol[4];
        // convert the rotation vector component to a quaternion
        rotvec_to_quat(chol_rotvec, quat_chol);
        
        // get quaternion parts of X and sigma points
        double *quat_X = &X[UKF_STATE_DIMENSION - 4];
        double *quat_sigma_plus = &sigma_plus[UKF_STATE_DIMENSION - 4];
        double *quat_sigma_minus = &sigma_minus[UKF_STATE_DIMENSION - 4];

        // quat_plus = quat_X * quat_chol
        quaternion_product_f64(quat_X, quat_chol, quat_sigma_plus);
        
        // quat_minus = quat_X * conj(quat_chol)
        double quat_chol_conj[4] = {quat_chol[0], -quat_chol[1], -quat_chol[2], -quat_chol[3]};
        quaternion_product_f64(quat_X, quat_chol_conj, quat_sigma_minus);
    }

    for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
        // call the state transition function on each row of the sigma points
        double *output_row = &sigmas_f[i * UKF_STATE_DIMENSION];
        ukfh->state_transition_function(sigma_points[i], dt, ukfh->flight_state, output_row);
    }
    return 0;
}

static int unscented_transform_f() {
    // set the temp residual matrices to intended sizes (unscented_transform_h reuses them and
    // changes sizes)
    temp_residuals.numCols = UKF_STATE_DIMENSION - 1;
    temp_residuals_transpose.numRows = UKF_STATE_DIMENSION - 1;
    temp_weighted_residuals_transpose.numRows = UKF_STATE_DIMENSION - 1;
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
        arm_scale_f64(&sigmas_f[i * UKF_STATE_DIMENSION], Wm[i], weighted_vector_sigmas, UKF_STATE_DIMENSION - 4);

        // quaternions can't be simply multiplied and summed like vector components, so they must
        // be converted to change in rotation, in rotation vector form, then scaled/summed like
        // normal, then converted back to quaternion in a "mean delta quaternion" form, then
        // "added" back to the current quaternion state to turn the delta quaternion into an
        // absolute orientation.

        // find the delta quaternion by multiplying by the conjugate of the current orientation
        double delta_quat[4];
        quaternion_product_f64(&sigmas_f[i * UKF_STATE_DIMENSION + (UKF_STATE_DIMENSION - 4)], quat_state_conj, delta_quat);
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
    quaternion_product_f64(mean_delta_quat, quat_state, &mean_x[UKF_STATE_DIMENSION - 4]);
    // copy the mean x vector into the X state vector
    memcpy(X, mean_x, sizeof(mean_x));
    // calculate and copy in the vector residuals into the resiudals matrix
    for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
        arm_sub_f64(&sigmas_f[i * UKF_STATE_DIMENSION], mean_x, &temp_residuals_data[i * (UKF_STATE_DIMENSION - 1)], UKF_STATE_DIMENSION - 4);

    }

    // compute updated P matrix
    for (int i = 0; i < UKF_STATE_DIMENSION - 1; i++) {
        for (int j = 0; j < UKF_NUM_SIGMAS; j++) {
            temp_residuals_transpose_data[i * UKF_NUM_SIGMAS + j] = temp_residuals_data[j * (UKF_STATE_DIMENSION - 1) + i];
        }
    }
    for (int i = 0; i < UKF_STATE_DIMENSION - 1; i++) {
        arm_mult_f64(&temp_residuals_transpose_data[i * UKF_NUM_SIGMAS], Wc, &temp_weighted_residuals_transpose_data[i * UKF_NUM_SIGMAS], UKF_NUM_SIGMAS);
    }
    
    int status = arm_mat_mult_f64(&temp_weighted_residuals_transpose, &temp_residuals, &P);
    if (status) {
        return 1;
    }
    return 0;
}

static int unscented_transform_h(double measurement_mean[UKF_MEASUREMENT_DIMENSION], arm_matrix_instance_f64 *innovation_cov) {
    // change all the temp residual matrix sizes to intended sizes (because unscented_transform_f
    // reuses them)
    temp_residuals.numCols = UKF_MEASUREMENT_DIMENSION;
    temp_residuals_transpose.numRows = UKF_MEASUREMENT_DIMENSION;
    temp_weighted_residuals_transpose.numRows = UKF_MEASUREMENT_DIMENSION;


    // set the resulting measurement mean to all 0's
    memset(measurement_mean, 0, sizeof(double) * UKF_MEASUREMENT_DIMENSION);
    double scaled_sigma_measurements[UKF_MEASUREMENT_DIMENSION];
    for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
        // scale row of measurement sigmas by weights
        arm_scale_f64(&sigmas_h[UKF_MEASUREMENT_DIMENSION * i], Wm[i], scaled_sigma_measurements, UKF_MEASUREMENT_DIMENSION);
        // sum all 43 rows of weighted measurement sigmas to the resulting measurement mean
        arm_add_f64(measurement_mean, scaled_sigma_measurements, measurement_mean, UKF_MEASUREMENT_DIMENSION);
    }

    // compute residuals, reuse residuals matrix from sigmas_f
    for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
        arm_sub_f64(&sigmas_h[i * UKF_MEASUREMENT_DIMENSION], measurement_mean, &temp_residuals_data[i * UKF_MEASUREMENT_DIMENSION], UKF_MEASUREMENT_DIMENSION);
    }

    // compute weighted residuals with Wc
    double weighted_residuals_data[UKF_NUM_SIGMAS * UKF_MEASUREMENT_DIMENSION];
    arm_matrix_instance_f64 weighted_residuals = {UKF_NUM_SIGMAS, UKF_MEASUREMENT_DIMENSION, weighted_residuals_data};
    for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
        arm_scale_f64(&temp_residuals_data[i * UKF_MEASUREMENT_DIMENSION], Wc[i], &weighted_residuals_data[i * UKF_MEASUREMENT_DIMENSION], UKF_MEASUREMENT_DIMENSION);
    }

    // transpose residuals
    if (arm_mat_trans_f64(&temp_residuals, &temp_residuals_transpose))
        return 1;

    // Compute P = residuals^T @ weighted_residuals
    if (arm_mat_mult_f64(&temp_residuals_transpose, &weighted_residuals, innovation_cov))
        return 1;

    // Add noise covariance R to P
    mat_add_f64(innovation_cov, &R, innovation_cov);
    return 0;
}

static int calculate_cross_covariance(const double *measurement_mean, arm_matrix_instance_f64 *P_cross_covariance) {
    // Reuse temp residual matrices
    temp_residuals.numRows = UKF_NUM_SIGMAS;
    temp_residuals.numCols = UKF_STATE_DIMENSION - 1;
    temp_residuals_transpose.numRows = UKF_STATE_DIMENSION - 1;
    temp_residuals_transpose.numCols = UKF_NUM_SIGMAS;

    // Local buffers for dz_res and weighted_dz (43 x 10)
    double dz_res_data[UKF_NUM_SIGMAS * UKF_MEASUREMENT_DIMENSION];
    double weighted_dz_data[UKF_NUM_SIGMAS * UKF_MEASUREMENT_DIMENSION];

    // Compute dx_res (residuals in tangent space) into temp_residuals
    for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
        const double *sigma_f_row = &sigmas_f[i * UKF_STATE_DIMENSION];
        double *dx_row = &temp_residuals_data[i * (UKF_STATE_DIMENSION - 1)];

        // Subtract vector part (first 18 elements)
        arm_sub_f64(sigma_f_row, X, dx_row, UKF_STATE_DIMENSION - 4);

        // Quaternion delta for rotvec part
        double quat_sig_f[4];
        for (int j = 0; j < 4; j++) {
            quat_sig_f[j] = sigma_f_row[UKF_STATE_DIMENSION - 4 + j];
        }
        double quat_mean_x[4];
        for (int j = 0; j < 4; j++) {
            quat_mean_x[j] = X[UKF_STATE_DIMENSION - 4 + j];
        }
        double conj_quat_x[4] = {quat_mean_x[0], -quat_mean_x[1], -quat_mean_x[2], -quat_mean_x[3]};
        double delta_quat[4];
        quaternion_product_f64(quat_sig_f, conj_quat_x, delta_quat);
        double delta_rotvec[3];
        quat_to_rotvec(delta_quat, delta_rotvec);

        // Copy rotvec to last 3 of dx_row
        memcpy(&dx_row[UKF_STATE_DIMENSION - 4], delta_rotvec, sizeof(double) * 3);
    }

    // Compute dz_res (43 x 10)
    for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
        const double *sigma_h_row = &sigmas_h[i * UKF_MEASUREMENT_DIMENSION];
        double *dz_row = &dz_res_data[i * UKF_MEASUREMENT_DIMENSION];
        arm_sub_f64(sigma_h_row, measurement_mean, dz_row, UKF_MEASUREMENT_DIMENSION);
    }

    // Weight dz rows by Wc
    for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
        arm_scale_f64(&dz_res_data[i * UKF_MEASUREMENT_DIMENSION], Wc[i], &weighted_dz_data[i * UKF_MEASUREMENT_DIMENSION], UKF_MEASUREMENT_DIMENSION);
    }
    arm_matrix_instance_f64 weighted_dz_mat = {UKF_NUM_SIGMAS, UKF_MEASUREMENT_DIMENSION, weighted_dz_data};

    // Transpose dx_res to dx_t (21 x 43) using temp_residuals_transpose
    for (int j = 0; j < UKF_STATE_DIMENSION - 1; j++) {
        for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
            temp_residuals_transpose_data[j * UKF_NUM_SIGMAS + i] = temp_residuals_data[i * (UKF_STATE_DIMENSION - 1) + j];
        }
    }
    
    // Compute P_cross = dx_t @ weighted_dz (21x43 @ 43x10 = 21x10)
    arm_status status = arm_mat_mult_f64(&temp_residuals_transpose, &weighted_dz_mat, P_cross_covariance);
    if (status != ARM_MATH_SUCCESS) {
        return 1;
    }
    return 0;
}

#ifdef TEST
double ukf_test_get_lambda(void) { return lambda_scaling_parameter; }
double* ukf_test_get_Wm(void) { return Wm; }
double* ukf_test_get_Wc(void) { return Wc; }
double* ukf_test_get_sigmas_f(void) { return sigmas_f; }
double* ukf_test_get_sigmas_h(void) { return sigmas_h; }
void ukf_test_get_sigma_points(double sigmas[UKF_NUM_SIGMAS][UKF_STATE_DIMENSION]) { memcpy(sigmas, sigma_points, UKF_NUM_SIGMAS * UKF_STATE_DIMENSION * sizeof(double));}
double* ukf_test_get_residuals(void) { return temp_residuals_data; }
double* ukf_test_get_weighted_vector_sigmas(void) { return weighted_vector_sigmas; }
double* ukf_test_get_Q_scaled(void) { return Q_scaled_data; }
double* ukf_test_get_S(void) { return innovation_data; }
double* ukf_test_get_pred_z(void) { return predicted_measurement; }
#endif