#pragma once

#include "eskf_config.h"
#include "matrix_helper.h"
#include <string.h>

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
 * @brief Predicted measurement z_pred = [pressure].
 *
 * @param x_nom         the nominal ESKF state vector
 * @param initial_altitude startup altitude computed from accumulated pressure
 * @param mag_world     unused (kept for API compatibility)
 * @param R_mag         unused (kept for API compatibility)
 * @param z_pred        output: predicted measurement vector
 */
void eskf_measurement_function(const float *x_nom, float initial_altitude,
                               const float *mag_world, const matrix_instance_f32 *R_mag,
                               float *z_pred);

/**
 * @brief Measurement Jacobian H (1x5) for pressure only.
 *
 * @param x_nom         the nominal ESKF state vector
 * @param initial_altitude startup altitude computed from accumulated pressure
 * @param mag_world     unused (kept for API compatibility)
 * @param R_mag         unused (kept for API compatibility)
 * @param H_data        output 5-element row-major array
 */
void eskf_measurement_jacobian(const float *x_nom, float initial_altitude, const float *mag_world,
                               const float *R_mag, float *H_data);
