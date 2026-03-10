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
 * @brief Transform raw IMU (sensor frame) to vehicle frame + convert units.
 *
 * accel: g-units → m/s²     gyro: deg/s → rad/s
 */
void eskf_imu_to_vehicle(const float accel_sensor[3], const float gyro_sensor[3],
                         float accel_vehicle_ms2[3], float gyro_vehicle_rads[3]);

/**
 * @brief Propagate the 10-dim nominal state (full strapdown).
 */
void eskf_nominal_predict(float x_nom[ESKF_NOMINAL_DIM],
                          const float u[ESKF_CONTROL_DIM], float dt);

/**
 * @brief Init-phase nominal predict: clamp pos/vel to zero, only integrate quaternion.
 */
void eskf_nominal_predict_init(float x_nom[ESKF_NOMINAL_DIM],
                               const float u[ESKF_CONTROL_DIM], float dt);

/**
 * @brief Build the 9×9 discrete error-state Jacobian F_d (running mode).
 * @param F_d_data  output 81-element row-major array
 */
void eskf_error_jacobian(const float x_nom[ESKF_NOMINAL_DIM],
                         const float u[ESKF_CONTROL_DIM], float dt,
                         float F_d_data[ESKF_ERROR_DIM * ESKF_ERROR_DIM]);

/**
 * @brief Init-phase error Jacobian: clamp pos/vel rows, only angular block.
 */
void eskf_error_jacobian_init(const float x_nom[ESKF_NOMINAL_DIM],
                              const float u[ESKF_CONTROL_DIM], float dt,
                              float F_d_data[ESKF_ERROR_DIM * ESKF_ERROR_DIM]);

/**
 * @brief Predicted measurement z_pred = [pressure, mag_sensor(3)].
 */
void eskf_measurement_function(const float x_nom[ESKF_NOMINAL_DIM],
                               float init_pressure,
                               const float mag_world[3],
                               float z_pred[ESKF_MEASUREMENT_DIM]);

/**
 * @brief Measurement Jacobian H (4×9).
 * @param H_data  output 36-element row-major array
 */
void eskf_measurement_jacobian(const float x_nom[ESKF_NOMINAL_DIM],
                               float init_pressure,
                               const float mag_world[3],
                               float H_data[ESKF_MEASUREMENT_DIM * ESKF_ERROR_DIM]);
