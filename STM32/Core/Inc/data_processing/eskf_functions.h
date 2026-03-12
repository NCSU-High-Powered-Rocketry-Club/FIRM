#pragma once

#include "eskf_config.h"
#include "matrix_helper.h"
#include <string.h>

/* ====================================================================
 * ESKF dynamics, measurement model, and Jacobians
 *
 * Ported from Python UKF/eskf_functions.py
 * ==================================================================== */

/**
 * @brief Transform raw IMU (sensor frame) to board frame + convert units.
 *
 * @param R_imu  3×3 row-major: IMU sensor → board frame rotation matrix
 * accel: g-units → m/s²     gyro: deg/s → rad/s
 */
void eskf_imu_to_vehicle(const float R_imu[9], const float accel_sensor[3],
                         const float gyro_sensor[3], float accel_vehicle_ms2[3],
                         float gyro_vehicle_rads[3]);

/**
 * @brief Propagate the 10-dim nominal state (full strapdown).
 *
 * @param R_imu  3×3 row-major: IMU sensor → board frame rotation matrix
 */
void eskf_nominal_predict(float x_nom[ESKF_NOMINAL_DIM], const float u[ESKF_CONTROL_DIM],
                          float dt, const float R_imu[9]);

/**
 * @brief Build the 9×9 discrete error-state Jacobian F_d.
 *
 * @param R_imu    3×3 row-major: IMU sensor → board frame rotation matrix
 * @param F_d_data output 81-element row-major array
 */
void eskf_error_jacobian(const float x_nom[ESKF_NOMINAL_DIM], const float u[ESKF_CONTROL_DIM],
                         float dt, const float R_imu[9],
                         float F_d_data[ESKF_ERROR_DIM * ESKF_ERROR_DIM]);

/**
 * @brief Predicted measurement z_pred = [pressure, mag_sensor(3)].
 *
 * @param R_mag  3×3 row-major: board frame → mag sensor rotation matrix
 */
void eskf_measurement_function(const float x_nom[ESKF_NOMINAL_DIM], float init_pressure,
                               const float mag_world[3], const float R_mag[9],
                               float z_pred[ESKF_MEASUREMENT_DIM]);

/**
 * @brief Measurement Jacobian H (4×9).
 *
 * @param R_mag  3×3 row-major: board frame → mag sensor rotation matrix
 * @param H_data output 36-element row-major array
 */
void eskf_measurement_jacobian(const float x_nom[ESKF_NOMINAL_DIM], float init_pressure,
                               const float mag_world[3], const float R_mag[9],
                               float H_data[ESKF_MEASUREMENT_DIM * ESKF_ERROR_DIM]);
