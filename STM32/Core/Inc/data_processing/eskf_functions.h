#pragma once

#include "eskf_config.h"
#include "matrix_helper.h"
#include <string.h>

/**
 * @brief Hardware-specific rotation matrices for sensors
 */
typedef struct {
  float R_imu_to_board[9];
  float R_mag_to_board[9];
} SensorOrientations_t;

/**
 * @brief Propagate the 6-dim nominal state.
 *
 * @param x_nom the nominal ESKF state vector
 * @param u     Control vector [accel(3), gyro(3)]
 * @param dt    Time step in seconds
 * @param R_imu 3x3 IMU sensor -> board frame rotation matrix
 */
void eskf_nominal_predict(float *x_nom, const float *u, float dt, const matrix_instance_f32 *R_imu);

/**
 * @brief Build the 5x5 error-state Jacobian F_d.
 *
 * @param x_nom    the nominal ESKF state vector
 * @param u        Control vector [accel(3), gyro(3)]
 * @param dt       Time step in seconds
 * @param R_imu    3x3 IMU sensor -> board frame rotation matrix
 * @param F_d_data output: 25-element row-major array
 */
void eskf_error_jacobian(const float *x_nom, const float *u, float dt,
                         const matrix_instance_f32 *R_imu, float *F_d_data);

/**
 * @brief Predicted measurement z_pred = [pressure, mag_sensor(3)].
 *
 * @param x_nom         the nominal ESKF state vector
 * @param init_pressure the ground-level initial pressure on filter init
 * @param mag_world     world-frame magnetic field vector, calculated on filter init
 * @param R_mag         3x3: board frame -> mag sensor rotation matrix
 * @param z_pred        output: predicted measurement vector
 */
void eskf_measurement_function(const float *x_nom, float init_pressure, const float *mag_world,
                               const matrix_instance_f32 *R_mag, float *z_pred);

/**
 * @brief Measurement Jacobian H (4x5).
 *
 * @param x_nom         the nominal ESKF state vector
 * @param init_pressure the ground-level initial pressure on filter init
 * @param mag_world     world-frame magnetic field vector, calculated on filter init
 * @param R_mag         3x3: board frame -> mag sensor rotation matrix
 * @param H_data        output 20-element row-major array
 */
void eskf_measurement_jacobian(const float *x_nom, float init_pressure, const float *mag_world,
                               const float *R_mag, float *H_data);
