#pragma once
#include "eskf_config.h"
#include "matrix_helper.h"
#include "state_machine.h"

#ifndef PI_F
#define PI_F 3.141592653589793F
#endif

/* ====================================================================
 * Error-State Extended Kalman Filter (ESKF)
 *
 * Ported from Python UKF/eskf.py
 * ==================================================================== */

typedef struct ESKF {
  /* ---- nominal state (10) ---- */
  float x_nom[ESKF_NOMINAL_DIM];

  /* ---- error-state covariance P (9×9 row-major) ---- */
  float P[ESKF_ERROR_DIM * ESKF_ERROR_DIM];

  /* ---- process & measurement noise (diag stored as full matrices) -- */
  float Q[ESKF_ERROR_DIM * ESKF_ERROR_DIM];
  float R[ESKF_MEASUREMENT_DIM * ESKF_MEASUREMENT_DIM];

  /* ---- sensor calibration ---- */
  float initial_pressure;
  float mag_world[3];

  /* ---- IMU biases (estimated during INIT, then fixed) ---- */
  float accel_bias[3];   /* in sensor frame, g-units */
  float gyro_bias[3];    /* in sensor frame, deg/s   */

  /* ---- accumulator for INIT phase bias estimation ---- */
  float accel_accum[3];
  float gyro_accum[3];
  uint32_t accum_count;

  /* ---- flight state ---- */
  ESKFFlightState flight_state;
  float elapsed_time;

  /* ---- measurement vector (set before calling update) ---- */
  float z[ESKF_MEASUREMENT_DIM];

  /* ---- debug / diagnostics ---- */
  float pred_z[ESKF_MEASUREMENT_DIM];
  float mahalanobis_dist;
} ESKF;

/**
 * @brief Initialise the ESKF struct: state, covariance, calibration.
 *
 * @param eskf              Pointer to ESKF struct
 * @param initial_pressure  Sea-level-ish pressure for barometric altitude
 * @param initial_accel     First accelerometer reading (sensor frame, g)
 * @param initial_mag       First magnetometer reading (sensor frame, normalised)
 * @return 0 on success
 */
int eskf_init(ESKF *eskf, float initial_pressure,
              const float *initial_accel, const float *initial_mag);

/**
 * @brief Accumulate a raw IMU sample during INIT phase (for bias estimation).
 */
void eskf_accumulate(ESKF *eskf, const float accel_raw[3], const float gyro_raw[3]);

/**
 * @brief ESKF prediction step (nominal propagation + covariance).
 *
 * @param eskf  Pointer to ESKF struct
 * @param u     Control vector [accel(3), gyro(3)] — sensor frame, bias-subtracted
 * @param dt    Time step in seconds
 */
void eskf_predict(ESKF *eskf, const float u[ESKF_CONTROL_DIM], float dt);

/**
 * @brief ESKF measurement update (pressure + mag).
 *
 * @param eskf  Pointer to ESKF struct
 * @param z     Measurement vector [pressure, mag_x, mag_y, mag_z]
 */
void eskf_update(ESKF *eskf, const float z[ESKF_MEASUREMENT_DIM]);

/**
 * @brief Set the measurement vector into eskf->z.
 */
void eskf_set_measurement(ESKF *eskf, const float *measurements);

/**
 * @brief Compute initial orientation from accel + mag and set mag_world.
 */
void calculate_initial_orientation(const float *imu_accel, const float *mag_field,
                                   float *init_quaternion, float *mag_world_frame);