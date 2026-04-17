#include "eskf_functions.h"
#include "matrix_helper.h"

#ifndef PI_F
#define PI_F 3.14159265358979323846F
#endif

/* ====================================================================
 * ESKF dynamics, measurement model, and Jacobians
 * ==================================================================== */

static void eskf_imu_to_board(const matrix_instance_f32 *R_imu, const float *accel_sensor,
                              const float *gyro_sensor, float *accel_vehicle_ms2,
                              float *gyro_vehicle_rads) {
  // Rotate imu -> vehicle
  mat_vec_mult_f32(R_imu, accel_sensor, accel_vehicle_ms2);
  mat_vec_mult_f32(R_imu, gyro_sensor, gyro_vehicle_rads);

  // convert units
  for (int i = 0; i < 3; ++i) {
    accel_vehicle_ms2[i] = accel_vehicle_ms2[i] * GRAVITY_METERS_PER_SECOND_SQUARED; // g to m/s^2
    gyro_vehicle_rads[i] = gyro_vehicle_rads[i] * (PI_F / 180.0F); // deg/sec to rad/sec
  }
}

void eskf_nominal_predict(float *x_nom, const float *u, float dt,
                          const matrix_instance_f32 *R_imu) {
  // convert sensor-frame imu measurements to board-frame
  float a_board_ms2[3], w_board_rads[3];
  eskf_imu_to_board(R_imu, u, u + 3, a_board_ms2, w_board_rads);

  // quaternion state and conjugate
  float *quat_state = &x_nom[ESKF_QUAT_W];
  float quat_conj[4] = {x_nom[ESKF_QUAT_W], -x_nom[ESKF_QUAT_X], -x_nom[ESKF_QUAT_Y],
                        -x_nom[ESKF_QUAT_Z]};

  // World-frame acceleration: (q * a * q_conj) − [0,0,g]
  float temp[4] = {0};
  float a_board_quat[4] = {0, a_board_ms2[0], a_board_ms2[1], a_board_ms2[2]};
  quaternion_product_f32(quat_state, a_board_quat, temp);
  quaternion_product_f32(temp, quat_conj, a_board_quat); // reusing a_board_quat
  float *a_world = &a_board_quat[1]; // a_world is the x,y,z element of the a_board_quat quaternion
  a_world[2] -= GRAVITY_METERS_PER_SECOND_SQUARED; // subtract from z axis

  // Integrate position and velocity
  x_nom[ESKF_POS_Z] += x_nom[ESKF_VEL_Z] * dt;
  x_nom[ESKF_VEL_Z] += a_world[2] * dt;

  // Quaternion integration: q = q * rotvec_to_quat(ω*dt)
  float delta_theta[3] = {w_board_rads[0] * dt, w_board_rads[1] * dt, w_board_rads[2] * dt};
  float delta_q[4];
  rotvec_to_quat(delta_theta, delta_q);

  float new_q[4];
  quaternion_product_f32(quat_state, delta_q, new_q);
  quaternion_normalize_f32(new_q);
  x_nom[ESKF_QUAT_W] = new_q[0];
  x_nom[ESKF_QUAT_X] = new_q[1];
  x_nom[ESKF_QUAT_Y] = new_q[2];
  x_nom[ESKF_QUAT_Z] = new_q[3];
}

void eskf_error_jacobian(const float *x_nom, const float *u, float dt,
                         const matrix_instance_f32 *R_imu, float *F_d_data) {
  // convert sensor-frame imu measurements to board-frame
  float a_board_ms2[3], w_board_rads[3];
  eskf_imu_to_board(R_imu, u, u + 3, a_board_ms2, w_board_rads);

  // get shorthands for quaternion states and control inputs
  const float qw = x_nom[ESKF_QUAT_W];
  const float qx = x_nom[ESKF_QUAT_X];
  const float qy = x_nom[ESKF_QUAT_Y];
  const float qz = x_nom[ESKF_QUAT_Z];

  float ax = a_board_ms2[0];
  float ay = a_board_ms2[1];
  float az = a_board_ms2[2];
  float wx = w_board_rads[0];
  float wy = w_board_rads[1];
  float wz = w_board_rads[2];

  memset(F_d_data, 0, ESKF_ERROR_DIM * ESKF_ERROR_DIM * sizeof(float));
  // set to identity matrix (1's on diagonal)
  for (int i = 0; i < ESKF_ERROR_DIM; i++) {
    F_d_data[i * ESKF_ERROR_DIM + i] = 1.0F;
  }

  // row 1: δẑ += δv_z * dt
  F_d_data[1] = dt;

  // directly compute the 3rd row of the rotation matrix from the quaternion
  float r31 = 2.0F * (qx * qz - qw * qy);
  float r32 = 2.0F * (qw * qz + qw * qx);
  float r33 = 1.0F - 2.0F * (qx * qx + qy * qy);

  // row 2: δv̇_z = (a_board_ms2 * r_3) * dt
  F_d_data[5 + 2] = (ay * r33 - az * r32) * dt;
  F_d_data[5 + 3] = (az * r31 - ax * r33) * dt;
  F_d_data[5 + 4] = (ax * r32 - ay * r31) * dt;

  // row 3: δθ̇ = -skew(ω) @ δθ * dt (Unrolled)
  F_d_data[10 + 3] = wz * dt;
  F_d_data[10 + 4] = -wy * dt;

  // row 4
  F_d_data[15 + 2] = -wz * dt;
  F_d_data[15 + 4] = wx * dt;

  // row 5
  F_d_data[20 + 2] = wy * dt;
  F_d_data[20 + 3] = -wx * dt;
}

void eskf_measurement_function(const float *x_nom, float initial_altitude,
                 const float *mag_world,
                               const matrix_instance_f32 *R_mag, float *z_pred) {
  // outputs predicted measurement: pressure only
  (void)mag_world;  // unused
  (void)R_mag;      // unused
  float altitude_meters = initial_altitude + x_nom[ESKF_POS_Z];

  // pressure from altitude
  z_pred[0] = PRESSURE_SEA_LEVEL_REFERENCE_PA *
        powf(1.0F - (altitude_meters / PRESSURE_ALTITUDE_CONST), PRESSURE_EXPONENT);
}

void eskf_measurement_jacobian(const float *x_nom, float initial_altitude,
                 const float *mag_world, const float *R_mag, float *H_data) {
  // measurement jacobian for pressure only
  (void)mag_world;  // unused
  (void)R_mag;      // unused
  memset(H_data, 0, ESKF_MEASUREMENT_DIM * ESKF_ERROR_DIM * sizeof(float));
  float altitude = initial_altitude + x_nom[ESKF_POS_Z];

  // ∂pressure/∂altitude
  H_data[0] = PRESSURE_SEA_LEVEL_REFERENCE_PA *
        (PRESSURE_EXPONENT * (-1.0F / PRESSURE_ALTITUDE_CONST)) *
        powf(1.0F - altitude / PRESSURE_ALTITUDE_CONST, PRESSURE_EXPONENT - 1.0F);
}
