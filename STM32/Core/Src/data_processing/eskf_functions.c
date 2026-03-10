#include "eskf_functions.h"

#ifndef PI_F
#define PI_F 3.14159265358979323846F
#endif

/* ====================================================================
 * ESKF dynamics, measurement model, and Jacobians
 *
 * Ported from Python UKF/eskf_functions.py
 * ==================================================================== */

/* ---- helper: 3×3 matrix-vector multiply (row-major) --------------- */
static void mat3_vec3_mult(const float M[9], const float v[3], float out[3]) {
  out[0] = M[0] * v[0] + M[1] * v[1] + M[2] * v[2];
  out[1] = M[3] * v[0] + M[4] * v[1] + M[5] * v[2];
  out[2] = M[6] * v[0] + M[7] * v[1] + M[8] * v[2];
}

/* ---- helper: 3×3 transpose-vector multiply (R^T @ v) -------------- */
static void mat3T_vec3_mult(const float M[9], const float v[3], float out[3]) {
  out[0] = M[0] * v[0] + M[3] * v[1] + M[6] * v[2];
  out[1] = M[1] * v[0] + M[4] * v[1] + M[7] * v[2];
  out[2] = M[2] * v[0] + M[5] * v[1] + M[8] * v[2];
}

/* ---- helper: 3×3 @ 3×3, both row-major ---------------------------- */
static void mat3_mat3_mult(const float A[9], const float B[9], float C[9]) {
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      float s = 0.0F;
      for (int k = 0; k < 3; ++k) {
        s += A[r * 3 + k] * B[k * 3 + c];
      }
      C[r * 3 + c] = s;
    }
  }
}

/* ---- helper: build 3×3 skew-symmetric into flat row-major --------- */
static void skew_flat(const float v[3], float S[9]) {
  S[0] =  0.0F;  S[1] = -v[2];  S[2] =  v[1];
  S[3] =  v[2];  S[4] =  0.0F;  S[5] = -v[0];
  S[6] = -v[1];  S[7] =  v[0];  S[8] =  0.0F;
}

/* ==================================================================== */

void eskf_imu_to_vehicle(const float accel_sensor[3], const float gyro_sensor[3],
                         float accel_vehicle_ms2[3], float gyro_vehicle_rads[3]) {
  /* Rotate sensor → vehicle */
  float a_rot[3], g_rot[3];
  mat3_vec3_mult(R_IMU_TO_VEHICLE, accel_sensor, a_rot);
  mat3_vec3_mult(R_IMU_TO_VEHICLE, gyro_sensor, g_rot);

  /* Convert units */
  for (int i = 0; i < 3; ++i) {
    accel_vehicle_ms2[i] = a_rot[i] * GRAVITY_METERS_PER_SECOND_SQUARED;  /* g → m/s² */
    gyro_vehicle_rads[i] = g_rot[i] * (PI_F / 180.0F);                    /* deg/s → rad/s */
  }
}

/* ==================================================================== */

void eskf_nominal_predict(float x_nom[ESKF_NOMINAL_DIM],
                          const float u[ESKF_CONTROL_DIM], float dt) {
  float a_veh[3], w_veh[3];
  eskf_imu_to_vehicle(u, u + 3, a_veh, w_veh);

  /* Quaternion → rotation matrix (vehicle→world) */
  float quat[4] = {x_nom[ESKF_QUAT_W], x_nom[ESKF_QUAT_X],
                    x_nom[ESKF_QUAT_Y], x_nom[ESKF_QUAT_Z]};
  quaternion_normalize_f32(quat);

  float R_v2w_data[9];
  arm_matrix_instance_f32 R_v2w = {3, 3, R_v2w_data};
  quat_to_rotation_matrix_f32(quat, &R_v2w);

  /* World-frame acceleration: R @ a_vehicle − [0,0,g] */
  float a_world[3];
  mat3_vec3_mult(R_v2w_data, a_veh, a_world);
  a_world[2] -= GRAVITY_METERS_PER_SECOND_SQUARED;

  /* Integrate position & velocity */
  x_nom[ESKF_POS_X] += x_nom[ESKF_VEL_X] * dt;
  x_nom[ESKF_POS_Y] += x_nom[ESKF_VEL_Y] * dt;
  x_nom[ESKF_POS_Z] += x_nom[ESKF_VEL_Z] * dt;

  x_nom[ESKF_VEL_X] += a_world[0] * dt;
  x_nom[ESKF_VEL_Y] += a_world[1] * dt;
  x_nom[ESKF_VEL_Z] += a_world[2] * dt;

  /* Quaternion integration: q ← q * rotvec_to_quat(ω*dt) */
  float delta_theta[3] = {w_veh[0] * dt, w_veh[1] * dt, w_veh[2] * dt};
  float delta_q[4];
  rotvec_to_quat(delta_theta, delta_q);

  float new_q[4];
  quaternion_product_f32(quat, delta_q, new_q);
  x_nom[ESKF_QUAT_W] = new_q[0];
  x_nom[ESKF_QUAT_X] = new_q[1];
  x_nom[ESKF_QUAT_Y] = new_q[2];
  x_nom[ESKF_QUAT_Z] = new_q[3];
}

/* ==================================================================== */

void eskf_nominal_predict_init(float x_nom[ESKF_NOMINAL_DIM],
                               const float u[ESKF_CONTROL_DIM], float dt) {
  float a_veh[3], w_veh[3];
  eskf_imu_to_vehicle(u, u + 3, a_veh, w_veh);
  (void)a_veh;  /* not used during init */

  /* Clamp translational states */
  x_nom[ESKF_POS_X] = 0.0F;
  x_nom[ESKF_POS_Y] = 0.0F;
  x_nom[ESKF_POS_Z] = 0.0F;
  x_nom[ESKF_VEL_X] = 0.0F;
  x_nom[ESKF_VEL_Y] = 0.0F;
  x_nom[ESKF_VEL_Z] = 0.0F;

  /* Quaternion integration */
  float quat[4] = {x_nom[ESKF_QUAT_W], x_nom[ESKF_QUAT_X],
                    x_nom[ESKF_QUAT_Y], x_nom[ESKF_QUAT_Z]};
  quaternion_normalize_f32(quat);

  float delta_theta[3] = {w_veh[0] * dt, w_veh[1] * dt, w_veh[2] * dt};
  float delta_q[4];
  rotvec_to_quat(delta_theta, delta_q);

  float new_q[4];
  quaternion_product_f32(quat, delta_q, new_q);
  x_nom[ESKF_QUAT_W] = new_q[0];
  x_nom[ESKF_QUAT_X] = new_q[1];
  x_nom[ESKF_QUAT_Y] = new_q[2];
  x_nom[ESKF_QUAT_Z] = new_q[3];
}

/* ==================================================================== */

void eskf_error_jacobian(const float x_nom[ESKF_NOMINAL_DIM],
                         const float u[ESKF_CONTROL_DIM], float dt,
                         float F_d_data[ESKF_ERROR_DIM * ESKF_ERROR_DIM]) {
  float a_veh[3], w_veh[3];
  eskf_imu_to_vehicle(u, u + 3, a_veh, w_veh);

  float quat[4] = {x_nom[ESKF_QUAT_W], x_nom[ESKF_QUAT_X],
                    x_nom[ESKF_QUAT_Y], x_nom[ESKF_QUAT_Z]};
  quaternion_normalize_f32(quat);

  float R_v2w_data[9];
  arm_matrix_instance_f32 R_v2w = {3, 3, R_v2w_data};
  quat_to_rotation_matrix_f32(quat, &R_v2w);

  /*
   * F_d = I(9) + F_c * dt   where F_c is:
   *   [0  I  0 ]          0-2: pos
   *   [0  0  -R@skew(a)]  3-5: vel
   *   [0  0  -skew(w) ]   6-8: theta
   */

  /* Start with identity */
  memset(F_d_data, 0, sizeof(float) * ESKF_ERROR_DIM * ESKF_ERROR_DIM);
  for (int i = 0; i < ESKF_ERROR_DIM; ++i) {
    F_d_data[i * ESKF_ERROR_DIM + i] = 1.0F;
  }

  /* F[0:3, 3:6] += I(3) * dt   (δṗ += δv * dt) */
  F_d_data[0 * ESKF_ERROR_DIM + 3] += dt;
  F_d_data[1 * ESKF_ERROR_DIM + 4] += dt;
  F_d_data[2 * ESKF_ERROR_DIM + 5] += dt;

  /* F[3:6, 6:9] = -R @ skew(a_vehicle) * dt */
  float S_a[9];
  skew_flat(a_veh, S_a);

  float RS_a[9];
  mat3_mat3_mult(R_v2w_data, S_a, RS_a);

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      F_d_data[(3 + r) * ESKF_ERROR_DIM + (6 + c)] = -RS_a[r * 3 + c] * dt;
    }
  }

  /* F[6:9, 6:9] += -skew(ω) * dt */
  float S_w[9];
  skew_flat(w_veh, S_w);

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      F_d_data[(6 + r) * ESKF_ERROR_DIM + (6 + c)] += -S_w[r * 3 + c] * dt;
    }
  }
}

/* ==================================================================== */

void eskf_error_jacobian_init(const float x_nom[ESKF_NOMINAL_DIM],
                              const float u[ESKF_CONTROL_DIM], float dt,
                              float F_d_data[ESKF_ERROR_DIM * ESKF_ERROR_DIM]) {
  float a_veh[3], w_veh[3];
  eskf_imu_to_vehicle(u, u + 3, a_veh, w_veh);
  (void)a_veh;

  memset(F_d_data, 0, sizeof(float) * ESKF_ERROR_DIM * ESKF_ERROR_DIM);
  for (int i = 0; i < ESKF_ERROR_DIM; ++i) {
    F_d_data[i * ESKF_ERROR_DIM + i] = 1.0F;
  }

  /* Clamp pos/vel error dynamics: F[0:3,0:3] = 0, F[3:6,3:6] = 0 */
  for (int i = 0; i < 3; ++i) {
    F_d_data[i * ESKF_ERROR_DIM + i] = 0.0F;
    F_d_data[(3 + i) * ESKF_ERROR_DIM + (3 + i)] = 0.0F;
  }

  /* Angular: F[6:9, 6:9] += -skew(ω) * dt */
  float S_w[9];
  skew_flat(w_veh, S_w);

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      F_d_data[(6 + r) * ESKF_ERROR_DIM + (6 + c)] += -S_w[r * 3 + c] * dt;
    }
  }
}

/* ==================================================================== */

void eskf_measurement_function(const float x_nom[ESKF_NOMINAL_DIM],
                               float init_pressure,
                               const float mag_world[3],
                               float z_pred[ESKF_MEASUREMENT_DIM]) {
  float altitude = x_nom[ESKF_POS_Z];

  /* Barometric pressure from altitude */
  float base = 1.0F - altitude / PRESSURE_ALTITUDE_CONST;
  if (base < 0.0F) base = 0.0F;
  z_pred[0] = init_pressure * powf(base, PRESSURE_EXPONENT);

  /* Quaternion → rotation matrix */
  float quat[4] = {x_nom[ESKF_QUAT_W], x_nom[ESKF_QUAT_X],
                    x_nom[ESKF_QUAT_Y], x_nom[ESKF_QUAT_Z]};
  quaternion_normalize_f32(quat);

  float R_v2w_data[9];
  arm_matrix_instance_f32 R_v2w = {3, 3, R_v2w_data};
  quat_to_rotation_matrix_f32(quat, &R_v2w);

  /* mag_vehicle = R_v2w^T @ mag_world */
  float mag_vehicle[3];
  mat3T_vec3_mult(R_v2w_data, mag_world, mag_vehicle);

  /* mag_sensor = R_VEHICLE_TO_MAG @ mag_vehicle */
  float mag_sensor[3];
  mat3_vec3_mult(R_VEHICLE_TO_MAG, mag_vehicle, mag_sensor);

  z_pred[1] = mag_sensor[0];
  z_pred[2] = mag_sensor[1];
  z_pred[3] = mag_sensor[2];
}

/* ==================================================================== */

void eskf_measurement_jacobian(const float x_nom[ESKF_NOMINAL_DIM],
                               float init_pressure,
                               const float mag_world[3],
                               float H_data[ESKF_MEASUREMENT_DIM * ESKF_ERROR_DIM]) {
  float altitude = x_nom[ESKF_POS_Z];

  memset(H_data, 0, sizeof(float) * ESKF_MEASUREMENT_DIM * ESKF_ERROR_DIM);

  /* ∂pressure/∂altitude → H[0, 2] */
  float base = 1.0F - altitude / PRESSURE_ALTITUDE_CONST;
  if (base > 0.0F) {
    float dp_dalt = init_pressure * PRESSURE_EXPONENT *
                    powf(base, PRESSURE_EXPONENT - 1.0F) *
                    (-1.0F / PRESSURE_ALTITUDE_CONST);
    H_data[0 * ESKF_ERROR_DIM + 2] = dp_dalt;
  }

  /* ∂mag_sensor/∂δθ → H[1:4, 6:9] = R_VEHICLE_TO_MAG @ skew(mag_vehicle) */
  float quat[4] = {x_nom[ESKF_QUAT_W], x_nom[ESKF_QUAT_X],
                    x_nom[ESKF_QUAT_Y], x_nom[ESKF_QUAT_Z]};
  quaternion_normalize_f32(quat);

  float R_v2w_data[9];
  arm_matrix_instance_f32 R_v2w = {3, 3, R_v2w_data};
  quat_to_rotation_matrix_f32(quat, &R_v2w);

  float mag_vehicle[3];
  mat3T_vec3_mult(R_v2w_data, mag_world, mag_vehicle);

  float S_mag[9];
  skew_flat(mag_vehicle, S_mag);

  /* R_VEHICLE_TO_MAG @ skew(mag_vehicle) */
  float block[9];
  mat3_mat3_mult(R_VEHICLE_TO_MAG, S_mag, block);

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      H_data[(1 + r) * ESKF_ERROR_DIM + (6 + c)] = block[r * 3 + c];
    }
  }
}
