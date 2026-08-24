#pragma once
#include "eskf_config.h"
#include "matrix_helper.h"

#ifndef PI_F
#define PI_F 3.141592653589793F
#endif

/* ====================================================================
 * Error-State Extended Kalman Filter (ESKF)
 * ==================================================================== */

typedef struct ESKF {
  // nominal state (6)
  float x_nom[ESKF_NOMINAL_DIM];

  // error-state covariance P (5x5 row-major)
  float P[ESKF_ERROR_DIM * ESKF_ERROR_DIM];

  // process & measurement noise (diag stored as full matrices)
  float Q[ESKF_ERROR_DIM * ESKF_ERROR_DIM];
  float R[ESKF_MEASUREMENT_DIM * ESKF_MEASUREMENT_DIM];

  // initial values/states
  float initial_pressure;
  float mag_world[3];
  float standby_acceleration[3];
  float standby_magnetic_field[3];
  float gyro_bias[3];
  float acceleration_norm_reference;
  float last_acceleration_norm;
  float last_angular_rate_norm;

  // Autonomous flight-phase and pressure-reliability estimates.  These are
  // deliberately outside the 5-state ESKF: they gate observations rather than
  // model continuously evolving vehicle dynamics.
  float launch_candidate_time_seconds;
  float launch_candidate_velocity;
  float landed_stationary_time_seconds;
  float coast_time_seconds;
  float last_dt_seconds;
  float filtered_pressure_altitude;
  float pressure_altitude_offset;
  float pressure_recovery_time_seconds;
  float pressure_reliable_time_seconds;
  float pressure_reliability;
  float pressure_coupling;
  float apogee_altitude;
  uint8_t launched;
  uint8_t launch_candidate_active;
  uint8_t standby_stationary;
  uint8_t coast_detected;
  uint8_t apogee_detected;
  uint8_t landed;
  uint8_t pressure_disturbed;

  // measurement vector (set before calling update)
  float z[ESKF_MEASUREMENT_DIM];
} ESKF;

typedef struct {
  double timestamp_seconds;
  float pressure_pascals;
  float raw_acceleration_x_gs;
  float raw_acceleration_y_gs;
  float raw_acceleration_z_gs;
  float raw_angular_rate_x_deg_per_s;
  float raw_angular_rate_y_deg_per_s;
  float raw_angular_rate_z_deg_per_s;
  float magnetic_field_x_microteslas;
  float magnetic_field_y_microteslas;
  float magnetic_field_z_microteslas;
} ESKFRawData;

/**
 * @brief Initialise the ESKF struct: state, covariance, initial values.
 *
 * @param eskf Pointer to ESKF struct
 * @return 0 on success
 */
int eskf_init(ESKF *eskf);

/**
 * @brief Accumulates raw pressure, acceleration, and magnetic field during intialization. Used
 * during init to calculate initial orientation and initial reference altitude.
 *
 * @param pressure_raw a data point with the current raw pressure
 * @param accel_raw a data point with the current raw acceleration (x, y, z)
 * @param mag_raw data point with the current raw magnetic fields (x, y, z)
 */
void eskf_accumulate(float pressure_raw, const float *accel_raw, const float *mag_raw);

/**
 * @brief ESKF prediction step (nominal propagation + covariance).
 *
 * @param eskf Pointer to ESKF struct
 * @param u Control vector [accel(3), gyro(3)] — calibrated sensor frame
 * @param dt Time step in seconds
 */
void eskf_predict(ESKF *eskf, const float u[ESKF_CONTROL_DIM], float dt);

/**
 * @brief ESKF measurement update (pressure + mag).
 *
 * @param eskf Pointer to ESKF struct
 */
void eskf_update(ESKF *eskf);

/**
 * @brief Set the measurement vector into eskf->z.
 */
void eskf_set_measurement(ESKF *eskf, const float *measurements);

/**
 * @brief Compute initial orientation from accel + mag and set mag_world.
 *
 * @param imu_accel Raw accelerometer reading (sensor frame)
 * @param mag_field Raw magnetometer reading (sensor frame)
 * @param R_imu 3x3 row-major: IMU sensor → board frame rotation
 * @param R_mag 3x3 row-major: board frame → mag sensor rotation
 */
void calculate_initial_orientation(const float *imu_accel, const float *mag_field,
                                   const float *R_imu, const float *R_mag,
                                  float *init_quaternion, float *mag_world_frame);
