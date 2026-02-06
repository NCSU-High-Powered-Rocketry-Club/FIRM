#include "unscented_kalman_filter.h"
#include "kalman_filter_config.h"
#include "usb_print_debug.h"


// static array allocation
static float X[UKF_STATE_DIMENSION]; // state vector
static float measurement_vector[UKF_MEASUREMENT_DIMENSION]; // raw measurements
static float P_data[UKF_COVARIANCE_DIMENSION * UKF_COVARIANCE_DIMENSION]; // covariance matrix
static float Q_data[UKF_COVARIANCE_DIMENSION * UKF_COVARIANCE_DIMENSION]; // process noise matrix
static float R_data[UKF_MEASUREMENT_DIMENSION * UKF_MEASUREMENT_DIMENSION]; // measurement noise matrix
static float sigma_points[UKF_NUM_SIGMAS][UKF_STATE_DIMENSION]; // base sigma point matrix
static float sigmas_f[UKF_NUM_SIGMAS * UKF_STATE_DIMENSION]; // process sigma point matrix
static float sigmas_h[UKF_NUM_SIGMAS * UKF_MEASUREMENT_DIMENSION]; // measurement sigma point matrix
static float cholesky_data[UKF_COVARIANCE_DIMENSION * UKF_COVARIANCE_DIMENSION];
static float Wm[UKF_NUM_SIGMAS]; // sigma point mean weights
static float Wc[UKF_NUM_SIGMAS]; // sigma point covariance weights
static float lambda_scaling_parameter;
static float predicted_measurement[UKF_MEASUREMENT_DIMENSION];
static float innovation_data[UKF_MEASUREMENT_DIMENSION * UKF_MEASUREMENT_DIMENSION];
static float kalman_gain_data[UKF_COVARIANCE_DIMENSION * UKF_MEASUREMENT_DIMENSION];
static float measurement_errors[UKF_MEASUREMENT_DIMENSION];
static float P_cross_covariance_data[UKF_COVARIANCE_DIMENSION * UKF_MEASUREMENT_DIMENSION];


static arm_matrix_instance_f32 P = {UKF_COVARIANCE_DIMENSION, UKF_COVARIANCE_DIMENSION, P_data};
static arm_matrix_instance_f32 Q = {UKF_COVARIANCE_DIMENSION, UKF_COVARIANCE_DIMENSION, Q_data};
static arm_matrix_instance_f32 R = {UKF_MEASUREMENT_DIMENSION, UKF_MEASUREMENT_DIMENSION, R_data};
static arm_matrix_instance_f32 cholesky = {UKF_COVARIANCE_DIMENSION, UKF_COVARIANCE_DIMENSION, cholesky_data};
static arm_matrix_instance_f32 innovation_covariance = {UKF_MEASUREMENT_DIMENSION, UKF_MEASUREMENT_DIMENSION, innovation_data};
static arm_matrix_instance_f32 P_cross_covariance = {UKF_COVARIANCE_DIMENSION, UKF_MEASUREMENT_DIMENSION, P_cross_covariance_data};
static arm_matrix_instance_f32 kalman_gain = {UKF_COVARIANCE_DIMENSION, UKF_MEASUREMENT_DIMENSION, kalman_gain_data};

// temp arrays
static float temp_residuals_data[UKF_NUM_SIGMAS * UKF_COVARIANCE_DIMENSION];
static float temp_residuals_transpose_data[UKF_COVARIANCE_DIMENSION * UKF_NUM_SIGMAS];
static float temp_weighted_residuals_transpose_data[UKF_COVARIANCE_DIMENSION * UKF_NUM_SIGMAS];
static float weighted_vector_sigmas[UKF_STATE_DIMENSION - 4];
static float temp_dz_residuals_data[UKF_NUM_SIGMAS * UKF_MEASUREMENT_DIMENSION];
static float temp_rotvec[3];
static float temp_rotvec2[3];
static float temp_quat[4];
static float temp_quat2[4];

// temp matrices
static arm_matrix_instance_f32 temp_residuals = {UKF_NUM_SIGMAS, UKF_COVARIANCE_DIMENSION, temp_residuals_data};
static arm_matrix_instance_f32 temp_residuals_transpose = {UKF_COVARIANCE_DIMENSION, UKF_NUM_SIGMAS, temp_residuals_transpose_data};
static arm_matrix_instance_f32 temp_weighted_residuals_transpose = {UKF_COVARIANCE_DIMENSION, UKF_NUM_SIGMAS, temp_weighted_residuals_transpose_data};

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
static int calculate_sigmas_f(UKF *ukfh, float dt);

static void ukf_clear_internal_buffers(void);

static int unscented_transform_f(void);

static int unscented_transform_h(float measurement_mean[UKF_MEASUREMENT_DIMENSION], arm_matrix_instance_f32 *innovation_cov);

static int calculate_cross_covariance(const float *measurement_mean, arm_matrix_instance_f32 *P_cross_covariance);


int ukf_init(UKF *ukfh, float initial_pressure, float* initial_acceleration, float* initial_magnetic_field) {
  ukf_clear_internal_buffers();

  // assign ukf struct to matrix values
  ukfh->X = X;
  ukfh->P = P.pData;
  ukfh->Q = Q.pData;
  ukfh->R = R.pData;
  ukfh->flight_state = &state;
  ukfh->measurement_errors = measurement_errors;
  ukfh->measurement_vector = measurement_vector;

  // initialize state machine (also initializes Q/R diagonals)
  init_state(ukfh);
  
  // set inital values of state vector, and initial diagonals of covariance matrix
  for (int i = 0; i < UKF_COVARIANCE_DIMENSION; i++) {
    X[i] = ukf_initial_state_estimate[i];
    P_data[i + UKF_COVARIANCE_DIMENSION * i] = ukf_initial_state_covariance_diag[i];
  }
  // set last element of X (because the length of X is one more than the length of P)
  X[UKF_STATE_DIMENSION - 1] = ukf_initial_state_estimate[UKF_STATE_DIMENSION - 1];
  // set initial quaternion, magnetometer, and pressure
  calculate_initial_orientation(initial_acceleration, initial_magnetic_field, &X[UKF_STATE_DIMENSION - 4], ukfh->mag_world);
  ukfh->initial_pressure = initial_pressure;
  // calculate ukf constants that are needed before running first predict and update
  calculate_sigma_weights();
  return 0;
}

static void ukf_clear_internal_buffers(void) {
  memset(X, 0, sizeof(X));
  memset(measurement_vector, 0, sizeof(measurement_vector));
  memset(P_data, 0, sizeof(P_data));
  memset(Q_data, 0, sizeof(Q_data));
  memset(R_data, 0, sizeof(R_data));

  memset(sigma_points, 0, sizeof(sigma_points));
  memset(sigmas_f, 0, sizeof(sigmas_f));
  memset(sigmas_h, 0, sizeof(sigmas_h));
  memset(cholesky_data, 0, sizeof(cholesky_data));

  memset(Wm, 0, sizeof(Wm));
  memset(Wc, 0, sizeof(Wc));
  lambda_scaling_parameter = 0.0F;

  memset(predicted_measurement, 0, sizeof(predicted_measurement));
  memset(innovation_data, 0, sizeof(innovation_data));
  memset(kalman_gain_data, 0, sizeof(kalman_gain_data));
  memset(P_cross_covariance_data, 0, sizeof(P_cross_covariance_data));
  memset(measurement_errors, 0, sizeof(measurement_errors));

  memset(temp_residuals_data, 0, sizeof(temp_residuals_data));
  memset(temp_residuals_transpose_data, 0, sizeof(temp_residuals_transpose_data));
  memset(temp_weighted_residuals_transpose_data, 0, sizeof(temp_weighted_residuals_transpose_data));
  memset(weighted_vector_sigmas, 0, sizeof(weighted_vector_sigmas));
  memset(temp_dz_residuals_data, 0, sizeof(temp_dz_residuals_data));

  memset(temp_rotvec, 0, sizeof(temp_rotvec));
  memset(temp_rotvec2, 0, sizeof(temp_rotvec2));
  memset(temp_quat, 0, sizeof(temp_quat));
  memset(temp_quat2, 0, sizeof(temp_quat2));

  // Defensive: these get mutated and reused between transforms.
  temp_residuals.numRows = UKF_NUM_SIGMAS;
  temp_residuals.numCols = UKF_COVARIANCE_DIMENSION;
  temp_residuals_transpose.numRows = UKF_COVARIANCE_DIMENSION;
  temp_residuals_transpose.numCols = UKF_NUM_SIGMAS;
  temp_weighted_residuals_transpose.numRows = UKF_COVARIANCE_DIMENSION;
  temp_weighted_residuals_transpose.numCols = UKF_NUM_SIGMAS;
}

int ukf_predict(UKF *ukfh, const float delta_time) {
  // check if dt is too fast
  if (delta_time < 1e-9F) {
    return 1;
  }

  // normalize the quaternion state
  quaternion_normalize_f32(&(ukfh->X[UKF_STATE_DIMENSION - 4]));
  int status = calculate_sigmas_f(ukfh, delta_time);
  if (status == 1) {
    // general error
    return 1;
  }
  status = unscented_transform_f();
  if (status) {
    // transform matrix multiply for P
    return 3;
  }
  return 0;
}

int ukf_update(UKF* ukfh) {
  // pass sigmas through the measurement function to determine how the predicted state would
  // be reflected in sensor measurements
  
  for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
    ukfh->measurement_function(&sigmas_f[i * UKF_STATE_DIMENSION], ukfh, &sigmas_h[i * UKF_MEASUREMENT_DIMENSION]);
  }
  
  // perform unscented transform to find mean of measurement sigma points, and the covariance of
  // the measurments
  if (unscented_transform_h(predicted_measurement, &innovation_covariance))
    return 1;

  // compute cross covariance between sigma points in state space (sigmas_f) and in measurement space (sigmas_h)
  if (calculate_cross_covariance(predicted_measurement, &P_cross_covariance)) {
    return 3;
  }

  float innovation_inverse_data[UKF_MEASUREMENT_DIMENSION * UKF_MEASUREMENT_DIMENSION];
  arm_matrix_instance_f32 innovation_covariance_inverse = {UKF_MEASUREMENT_DIMENSION, UKF_MEASUREMENT_DIMENSION, innovation_inverse_data};
  mat_inverse_f32(&innovation_covariance, &innovation_covariance_inverse);
  // calculate the kalman gain, which defines how much to weigh the prediction by compared
  // to the sensor measurements
  mat_mult_f32(&P_cross_covariance, &innovation_covariance_inverse, &kalman_gain);
  
  // calculate measurement error metric for debug and state change purposes. These values can
  // be used to determine when a measurement is outside of the expected range and can help tune
  // the filter, remove outliers, or detecting state changes
  float measurement_residuals[UKF_MEASUREMENT_DIMENSION];
  vec_sub_f32(ukfh->measurement_vector, predicted_measurement, measurement_residuals, UKF_MEASUREMENT_DIMENSION);
  mat_vec_mult_f32(&innovation_covariance_inverse, measurement_residuals, ukfh->measurement_errors);
  vec_mult_f32(measurement_residuals, ukfh->measurement_errors, ukfh->measurement_errors, UKF_MEASUREMENT_DIMENSION);
  
  // calculate the change in the state vector based on kalman gain
  float delta_x[UKF_COVARIANCE_DIMENSION];
  mat_vec_mult_f32(&kalman_gain, measurement_residuals, delta_x);
  // if we are not in standby state, do not change the accelerometer/gyroscope offsets at all

  // extract rotation vector component of delta_x and change to delta quaternion
  rotvec_to_quat(&delta_x[UKF_STATE_DIMENSION - 4], temp_quat);
  // update state vector by multiplying the delta quaternion and adding the delta x vector components
  quaternion_product_f32(temp_quat, &X[UKF_STATE_DIMENSION - 4], &X[UKF_STATE_DIMENSION - 4]);
  vec_add_f32(X, delta_x, X, UKF_STATE_DIMENSION - 4);
  // compute next covariance matrix, P
  // new_P = P - (kalman_gain * innovation_covariance * kalman_gain^T)
  float kalman_gain_transpose_data[UKF_MEASUREMENT_DIMENSION * UKF_COVARIANCE_DIMENSION];
  arm_matrix_instance_f32 kalman_gain_transpose = {UKF_MEASUREMENT_DIMENSION, UKF_COVARIANCE_DIMENSION, kalman_gain_transpose_data};
  mat_trans_f32(&kalman_gain, &kalman_gain_transpose);
  float temp_matrix_data[UKF_MEASUREMENT_DIMENSION * UKF_COVARIANCE_DIMENSION];
  arm_matrix_instance_f32 temp_matrix = {UKF_MEASUREMENT_DIMENSION, UKF_COVARIANCE_DIMENSION, temp_matrix_data};
  mat_mult_f32(&innovation_covariance, &kalman_gain_transpose, &temp_matrix);
  float negative_delta_P_data[UKF_COVARIANCE_DIMENSION * UKF_COVARIANCE_DIMENSION];
  arm_matrix_instance_f32 negative_delta_P = {UKF_COVARIANCE_DIMENSION, UKF_COVARIANCE_DIMENSION, negative_delta_P_data};
  mat_mult_f32(&kalman_gain, &temp_matrix, &negative_delta_P);
  mat_sub_f32(&P, &negative_delta_P, &P);
  
  // update state machine
  state_update(ukfh);
  return 0;
}

static void calculate_sigma_weights(void) {
  lambda_scaling_parameter = UKF_SIGMA_SPREAD_ALPHA * UKF_SIGMA_SPREAD_ALPHA * (UKF_STATE_DIMENSION - 1.0F + UKF_SIGMA_TERTIARY_KAPPA) - (UKF_STATE_DIMENSION - 1.0F);
  float weight_component = 1.0F / (2.0F * (lambda_scaling_parameter + UKF_STATE_DIMENSION - 1.0F));
  for (int i = 1; i < UKF_NUM_SIGMAS; i++) {
    Wm[i] = weight_component;
    Wc[i] = weight_component;
  }
  Wm[0] = lambda_scaling_parameter / (UKF_STATE_DIMENSION - 1.0F + lambda_scaling_parameter);
  Wc[0] = lambda_scaling_parameter / (UKF_STATE_DIMENSION - 1.0F + lambda_scaling_parameter) + (1.0F - UKF_SIGMA_SPREAD_ALPHA * UKF_SIGMA_SPREAD_ALPHA + UKF_SIGMA_WEIGHT_BETA);
}

static int calculate_sigmas_f(UKF *ukfh, float dt) {
  // force P to be symmetric
  if (symmetrize(&P)) {
    return 1;
  }
  float Q_scaled_data[UKF_COVARIANCE_DIMENSION * UKF_COVARIANCE_DIMENSION];
  arm_matrix_instance_f32 Q_scaled = {UKF_COVARIANCE_DIMENSION, UKF_COVARIANCE_DIMENSION, Q_scaled_data};
  // calculate cholesky square root with added noise
  mat_scale_f32(&Q, dt, &Q_scaled);
  mat_add_f32(&P, &Q_scaled, &Q_scaled);
  mat_scale_f32(&Q_scaled, lambda_scaling_parameter + UKF_COVARIANCE_DIMENSION, &Q_scaled);
  mat_cholesky_f32(&Q_scaled, &cholesky);

  // first sigma point is just the state, X
  memcpy(sigma_points[0], X, sizeof(float) * UKF_STATE_DIMENSION);

  // build remaining sigma points
  for (int i = 0; i < UKF_COVARIANCE_DIMENSION; i++) {
    
    // get indices in sigma_f data for the (i'th + 1) row and (i'th + 16'th) row
    float *sigma_plus = sigma_points[i + 1];
    float *sigma_minus = sigma_points[i + UKF_STATE_DIMENSION];

    // copy X into both sigmas
    memcpy(sigma_plus, X, UKF_STATE_DIMENSION * sizeof(float));
    memcpy(sigma_minus, X, UKF_STATE_DIMENSION * sizeof(float));

    // add or subtract the cholesky column to the vector part of the sigma rows
    for (int k = 0; k < UKF_STATE_DIMENSION - 4; k++) {
      float cholesky_value = cholesky_data[k*UKF_COVARIANCE_DIMENSION + i];
      sigma_plus[k] += cholesky_value;
      sigma_minus[k] -= cholesky_value;
    }
    
    // extract the rotation vector component of the cholesky columns for this loop
    float *chol_rotvec = temp_rotvec;
    chol_rotvec[0] = cholesky_data[UKF_COVARIANCE_DIMENSION * (UKF_STATE_DIMENSION - 4) + i];
    chol_rotvec[1] = cholesky_data[UKF_COVARIANCE_DIMENSION * (UKF_STATE_DIMENSION - 3) + i];
    chol_rotvec[2] = cholesky_data[UKF_COVARIANCE_DIMENSION * (UKF_STATE_DIMENSION - 2) + i];

    // get quaternion parts of X and sigma points
    float *quat_X = &X[UKF_STATE_DIMENSION - 4];
    float *quat_sigma_plus = &sigma_plus[UKF_STATE_DIMENSION - 4];
    float *quat_sigma_minus = &sigma_minus[UKF_STATE_DIMENSION - 4];

    // convert the rotation vector component to a quaternion
    rotvec_to_quat(chol_rotvec, quat_sigma_plus);
    // quat_minus = quat_X * conj(cholesky quat)
    quat_sigma_minus[0] = quat_sigma_plus[0];
    quat_sigma_minus[1] = -quat_sigma_plus[1];
    quat_sigma_minus[2] = -quat_sigma_plus[2];
    quat_sigma_minus[3] = -quat_sigma_plus[3];
    quaternion_product_f32(quat_X, quat_sigma_minus, quat_sigma_minus);
    // quat_plus = quat_X * cholesky quat
    quaternion_product_f32(quat_X, quat_sigma_plus, quat_sigma_plus);
  }

  for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
    // call the state transition function on each row of the sigma points
    float *output_row = &sigmas_f[i * UKF_STATE_DIMENSION];
    ukfh->state_transition_function(sigma_points[i], dt, *(ukfh->flight_state), output_row);
  }
  return 0;
}

static int unscented_transform_f() {
  // set the temp residual matrices to intended sizes (unscented_transform_h reuses them and
  // changes sizes)
  temp_residuals.numCols = UKF_COVARIANCE_DIMENSION;
  temp_residuals_transpose.numRows = UKF_COVARIANCE_DIMENSION;
  temp_weighted_residuals_transpose.numRows = UKF_COVARIANCE_DIMENSION;
  temp_weighted_residuals_transpose.numCols = UKF_NUM_SIGMAS;
  // initialize the sigma means vector as all 0
  float mean_x[UKF_STATE_DIMENSION - 4];
  memset(mean_x, 0, sizeof(mean_x));
  float *mean_delta_rotvec = temp_rotvec;
  mean_delta_rotvec[0] = 0.0F;
  mean_delta_rotvec[1] = 0.0F;
  mean_delta_rotvec[2] = 0.0F;
  // get the current quaternion state and it's conjugate
  float *quat_state = &X[UKF_STATE_DIMENSION - 4];

  temp_quat[0] = quat_state[0];
  temp_quat[1] = -quat_state[1];
  temp_quat[2] = -quat_state[2];
  temp_quat[3] = -quat_state[3];
  float *quat_state_conj = temp_quat;

  // iterate through sigma points, multiplying by the weights to find the mean of each state
  for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
      
    // scale the vector components by Wm
    vec_scale_f32(&sigmas_f[i * UKF_STATE_DIMENSION], Wm[i], weighted_vector_sigmas, UKF_STATE_DIMENSION - 4);

    // quaternions can't be simply multiplied and summed like vector components, so they must
    // be converted to change in rotation, in rotation vector form, then scaled/summed like
    // normal, then converted back to quaternion in a "mean delta quaternion" form, then
    // "added" back to the current quaternion state to turn the delta quaternion into an
    // absolute orientation.

    // find the delta quaternion by multiplying by the conjugate of the current orientation
    float *delta_quat = temp_quat2;
    quaternion_product_f32(&sigmas_f[i * UKF_STATE_DIMENSION + (UKF_STATE_DIMENSION - 4)], quat_state_conj, delta_quat);
    // convert the delta quaternions to delta rotation vectors
    float delta_rotvec_sigmas[3];
    quat_to_rotvec(delta_quat, delta_rotvec_sigmas);

    // scale delta rotation vectors by Wm for mean computation only
    float weighted_delta_rotvec_sigmas[3];
    vec_scale_f32(delta_rotvec_sigmas, Wm[i], weighted_delta_rotvec_sigmas, 3);

    // add the vector and rotvec components to the mean vectors
    for (int j = 0; j < (UKF_STATE_DIMENSION - 4); j++) {
      mean_x[j] += weighted_vector_sigmas[j];
    }
    mean_delta_rotvec[0] += weighted_delta_rotvec_sigmas[0];
    mean_delta_rotvec[1] += weighted_delta_rotvec_sigmas[1];
    mean_delta_rotvec[2] += weighted_delta_rotvec_sigmas[2];

    // set the last 3 elements of the row in the residuals matrix to the UNWEIGHTED delta rotvec
    memcpy(&temp_residuals_data[i * UKF_COVARIANCE_DIMENSION + (UKF_STATE_DIMENSION - 4)], delta_rotvec_sigmas, sizeof(float) * 3);
  }

  // convert the mean delta rotation vector into a mean delta quaternion, then multiply the
  // delta quaternion to the current quaternion state to find the mean quaternion
  float *mean_delta_quat = temp_quat2;
  rotvec_to_quat(mean_delta_rotvec, mean_delta_quat);
  quaternion_product_f32(mean_delta_quat, quat_state, quat_state);
  quaternion_normalize_f32(quat_state);
  // copy the mean x vector into the X state vector
  memcpy(X, mean_x, sizeof(mean_x));
  // calculate and copy in the vector residuals into the resiudals matrix
  for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
    vec_sub_f32(&sigmas_f[i * UKF_STATE_DIMENSION], X, &temp_residuals_data[i * UKF_COVARIANCE_DIMENSION], UKF_STATE_DIMENSION - 4);
  }

  // compute updated P matrix
  for (int i = 0; i < UKF_COVARIANCE_DIMENSION; i++) {
    for (int j = 0; j < UKF_NUM_SIGMAS; j++) {
      temp_residuals_transpose_data[i * UKF_NUM_SIGMAS + j] = temp_residuals_data[j * UKF_COVARIANCE_DIMENSION + i];
    }
  }
  for (int i = 0; i < UKF_COVARIANCE_DIMENSION; i++) {
    vec_mult_f32(&temp_residuals_transpose_data[i * UKF_NUM_SIGMAS], Wc, &temp_weighted_residuals_transpose_data[i * UKF_NUM_SIGMAS], UKF_NUM_SIGMAS);
  }
  
  mat_mult_f32(&temp_weighted_residuals_transpose, &temp_residuals, &P);
  
  return 0;
}

static int unscented_transform_h(float measurement_mean[UKF_MEASUREMENT_DIMENSION], arm_matrix_instance_f32 *innovation_cov) {
  // change all the temp residual matrix sizes to intended sizes (because unscented_transform_f
  // reuses them)
  temp_residuals.numCols = UKF_MEASUREMENT_DIMENSION;
  temp_residuals.numRows = UKF_NUM_SIGMAS;
  temp_residuals_transpose.numRows = UKF_MEASUREMENT_DIMENSION;
  temp_weighted_residuals_transpose.numRows = UKF_NUM_SIGMAS;
  temp_weighted_residuals_transpose.numCols = UKF_MEASUREMENT_DIMENSION;
  arm_matrix_instance_f32 *temp_weighted_residuals = &temp_weighted_residuals_transpose;


  // set the resulting measurement mean to all 0's
  memset(measurement_mean, 0, sizeof(float) * UKF_MEASUREMENT_DIMENSION);
  float scaled_sigma_measurements[UKF_MEASUREMENT_DIMENSION];
  for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
    // scale row of measurement sigmas by weights
    vec_scale_f32(&sigmas_h[UKF_MEASUREMENT_DIMENSION * i], Wm[i], scaled_sigma_measurements, UKF_MEASUREMENT_DIMENSION);
    // sum all rows of weighted measurement sigmas to the resulting measurement mean
    vec_add_f32(measurement_mean, scaled_sigma_measurements, measurement_mean, UKF_MEASUREMENT_DIMENSION);
  }

  // compute residuals, reuse residuals matrix from sigmas_f
  for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
    vec_sub_f32(&sigmas_h[i * UKF_MEASUREMENT_DIMENSION], measurement_mean, &temp_residuals_data[i * UKF_MEASUREMENT_DIMENSION], UKF_MEASUREMENT_DIMENSION);
    // compute weighted residuals with Wc
    vec_scale_f32(&temp_residuals_data[i * UKF_MEASUREMENT_DIMENSION], Wc[i], &temp_weighted_residuals->pData[i * UKF_MEASUREMENT_DIMENSION], UKF_MEASUREMENT_DIMENSION);
  }

  // transpose residuals
  mat_trans_f32(&temp_residuals, &temp_residuals_transpose);

  // Compute P = residuals^T @ weighted_residuals
  mat_mult_f32(&temp_residuals_transpose, temp_weighted_residuals, innovation_cov);

  // Add noise covariance R to P
  mat_add_f32(innovation_cov, &R, innovation_cov);
  return 0;
}

static int calculate_cross_covariance(const float *measurement_mean, arm_matrix_instance_f32 *P_cross_covariance) {
  // Reuse temp residual matrices
  temp_residuals.numRows = UKF_NUM_SIGMAS;
  temp_residuals.numCols = UKF_COVARIANCE_DIMENSION;
  temp_residuals_transpose.numRows = UKF_COVARIANCE_DIMENSION;
  temp_residuals_transpose.numCols = UKF_NUM_SIGMAS;
  temp_weighted_residuals_transpose.numRows = UKF_NUM_SIGMAS;
  temp_weighted_residuals_transpose.numCols = UKF_MEASUREMENT_DIMENSION;
  arm_matrix_instance_f32 *weighted_dz = &temp_weighted_residuals_transpose;

  // Compute dx_res (residuals in tangent space) into temp_residuals
  for (int i = 0; i < UKF_NUM_SIGMAS; i++) {

    // Subtract vector part (first 18 elements)
    vec_sub_f32(&sigmas_f[i * UKF_STATE_DIMENSION], X, &temp_residuals_data[i * (UKF_STATE_DIMENSION - 1)], UKF_STATE_DIMENSION - 4);

    // Quaternion delta for rotvec part
    float *conj_quat_x = temp_quat;
    conj_quat_x[0] = X[UKF_STATE_DIMENSION - 4];
    conj_quat_x[1] = -X[UKF_STATE_DIMENSION - 3];
    conj_quat_x[2] = -X[UKF_STATE_DIMENSION - 2];
    conj_quat_x[3] = -X[UKF_STATE_DIMENSION - 1];
    float* delta_quat = temp_quat2;
    quaternion_product_f32(&sigmas_f[i * UKF_STATE_DIMENSION + UKF_STATE_DIMENSION - 4], conj_quat_x, delta_quat);
    float *delta_rotvec = temp_rotvec;
    quat_to_rotvec(delta_quat, delta_rotvec);

    // Copy rotvec to last 3 of dx_row
    memcpy(&temp_residuals_data[i * (UKF_STATE_DIMENSION - 1) + UKF_STATE_DIMENSION - 4], delta_rotvec, sizeof(float) * 3);
  }

  // Compute dz_res (43 x 10)
  for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
    const float *sigma_h_row = &sigmas_h[i * UKF_MEASUREMENT_DIMENSION];
    vec_sub_f32(sigma_h_row, measurement_mean, &temp_dz_residuals_data[i * UKF_MEASUREMENT_DIMENSION], UKF_MEASUREMENT_DIMENSION);
  }

  // Weight dz rows by Wc
  for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
    vec_scale_f32(&temp_dz_residuals_data[i * UKF_MEASUREMENT_DIMENSION], Wc[i], &weighted_dz->pData[i * UKF_MEASUREMENT_DIMENSION], UKF_MEASUREMENT_DIMENSION);
  }

  // Transpose dx_res to dx_t using temp_residuals_transpose
  for (int j = 0; j < UKF_COVARIANCE_DIMENSION; j++) {
    for (int i = 0; i < UKF_NUM_SIGMAS; i++) {
      temp_residuals_transpose_data[j * UKF_NUM_SIGMAS + i] = temp_residuals_data[i * UKF_COVARIANCE_DIMENSION + j];
    }
  }
  
  // Compute P_cross = dx_t @ weighted_dz (21x43 @ 43x10 = 21x10)
  mat_mult_f32(&temp_residuals_transpose, weighted_dz, P_cross_covariance);
  return 0;
}

void calculate_initial_orientation(const float *imu_accel, const float *mag_field, float *init_quaternion, float *mag_world_frame) {
  float norm_acc = sqrtf(imu_accel[0] * imu_accel[0] + imu_accel[1] * imu_accel[1] + imu_accel[2] * imu_accel[2]);
  float norm_mag = sqrtf(mag_field[0] * mag_field[0] + mag_field[1] * mag_field[1] + mag_field[2] * mag_field[2]);
  float acc_vehicle[3] = {
    (imu_accel[0] / SQRT2_F - imu_accel[1] / SQRT2_F) / norm_acc,
    (imu_accel[0] / SQRT2_F + imu_accel[1] / SQRT2_F) / norm_acc,
    imu_accel[2] / norm_acc,
  };
  float *mag_vehicle_quat = temp_quat;
  mag_vehicle_quat[0] = 0.0F;
  mag_vehicle_quat[1] = mag_field[0] / norm_mag;
  mag_vehicle_quat[2] = mag_field[1] / norm_mag;
  mag_vehicle_quat[3] = -mag_field[2] / norm_mag;

  float roll = atan2f(acc_vehicle[1], acc_vehicle[2]);
  float pitch = atan2f(-acc_vehicle[0], sqrtf(acc_vehicle[1] * acc_vehicle[1] + acc_vehicle[2] * acc_vehicle[2]));
  float mx2 = mag_vehicle_quat[1] * cosf(pitch) + mag_vehicle_quat[3] * sinf(pitch);
  float my2 = mag_vehicle_quat[1] * sinf(roll) * sinf(pitch) + mag_vehicle_quat[2] * cosf(roll) - mag_vehicle_quat[3] * sinf(roll) * cosf(pitch);
  float yaw = atan2f(-my2, mx2);

  init_quaternion[0] = cosf(roll * 0.5F) * cosf(pitch * 0.5F) * cosf(yaw * 0.5F) + sinf(roll * 0.5F) * sinf(pitch * 0.5F) * sinf(yaw * 0.5F);
  init_quaternion[1] = sinf(roll * 0.5F) * cosf(pitch * 0.5F) * cosf(yaw * 0.5F) - cosf(roll * 0.5F) * sinf(pitch * 0.5F) * sinf(yaw * 0.5F);
  init_quaternion[2] = cosf(roll * 0.5F) * sinf(pitch * 0.5F) * cosf(yaw * 0.5F) + sinf(roll * 0.5F) * cosf(pitch * 0.5F) * sinf(yaw * 0.5F);
  init_quaternion[3] = cosf(roll * 0.5F) * cosf(pitch * 0.5F) * sinf(yaw * 0.5F) - sinf(roll * 0.5F) * sinf(pitch * 0.5F) * cosf(yaw * 0.5F);

  float quat_conj[4] = {init_quaternion[0], -init_quaternion[1], -init_quaternion[2], -init_quaternion[3]};
  float temp[4];
  float mag_world[4];
  quaternion_product_f32(init_quaternion, mag_vehicle_quat, temp);
  quaternion_product_f32(temp, quat_conj, mag_world);
  mag_world_frame[0] = mag_world[1];
  mag_world_frame[1] = mag_world[2];
  mag_world_frame[2] = mag_world[3];
}

void ukf_set_measurement(UKF *ukfh, const float *measurements) {
  // normalize magnetometer measurements
  const float *mag_field = &measurements[UKF_MEASUREMENT_DIMENSION - 3];
  const float norm_mag = sqrtf(mag_field[0] * mag_field[0] + mag_field[1] * mag_field[1] + mag_field[2] * mag_field[2]);
  // copy over the measurements, and normalize magnetometer
  memcpy(ukfh->measurement_vector, measurements, sizeof(measurements[0]) * (UKF_MEASUREMENT_DIMENSION - 3));
  if (norm_mag < 1.0F)
    return;
  ukfh->measurement_vector[UKF_MEASUREMENT_DIMENSION - 3] = mag_field[0] / norm_mag;
  ukfh->measurement_vector[UKF_MEASUREMENT_DIMENSION - 2] = mag_field[1] / norm_mag;
  ukfh->measurement_vector[UKF_MEASUREMENT_DIMENSION - 1] = mag_field[2] / norm_mag;
}


#ifdef TEST
float ukf_test_get_lambda(void) { return lambda_scaling_parameter; }
float* ukf_test_get_Wm(void) { return Wm; }
float* ukf_test_get_Wc(void) { return Wc; }
float* ukf_test_get_sigmas_f(void) { return sigmas_f; }
float* ukf_test_get_sigmas_h(void) { return sigmas_h; }
void ukf_test_get_sigma_points(float sigmas[UKF_NUM_SIGMAS][UKF_STATE_DIMENSION]) { memcpy(sigmas, sigma_points, UKF_NUM_SIGMAS * UKF_STATE_DIMENSION * sizeof(float));}
float* ukf_test_get_residuals(void) { return temp_residuals_data; }
float* ukf_test_get_S(void) { return innovation_data; }
float* ukf_test_get_pred_z(void) { return predicted_measurement; }
float* ukf_test_get_cholesky(void) {return cholesky_data;}
#endif