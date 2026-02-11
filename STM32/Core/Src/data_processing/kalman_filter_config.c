#include "kalman_filter_config.h"

// This function takes in altitude and returns a value for pressure.
// Using data from https://www.engineeringtoolbox.com/standard-atmosphere-d_604.html
// Plotting the data point's, we found a relationship and that is what the function
// Uses to calculate pressure. This is super doo doo, please don't murder us.
float altitude_To_Air_Density_Table(float altitude) {
  float pressure;
  // ad =-0.0807*altitude+1.2539
  pressure = -0.0807F * altitude + 1.2539F;
  return pressure;
}

// clang-format off
float ukf_initial_state_estimate[UKF_STATE_DIMENSION] = {
    0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F,
    0.0F, 0.0F, 0.0F,
    1.0F, 0.0F, 0.0F, 0.0F
};

float ukf_initial_state_covariance_diag[UKF_COVARIANCE_DIMENSION] = {
    1e-6F, 1e-6F, 1e-6F,
    1e-6F, 1e-6F, 1e-6F,
    1e-2F, 1e-2F, 1e-2F,
    1e-5F, 1e-5F, 1e-5F,
    1e-1F, 1e-1F, 1e-1F,
};

float ukf_state_process_covariance_diag[STATE_MACHINE_NUM_STATES][UKF_COVARIANCE_DIMENSION] = {
    { // Standby
        1.0F, 1.0F, 1.0F, // position (x, y, z)
        1.0F, 1.0F, 1.0F, // velocity (x, y, z)
        1e-3F, 1e-3F, 1e-3F, // acceleration (x, y, z)
        1.0F, 1.0F, 1.0F, // gyro (x, y, z)
        1e-1F, 1e-1F, 1e-1F // orientation (r, p, y)
    },
    { // Motor Burn
        5e-2F, 5e-2F, 5e-2F, // position (x, y, z)
        5e-2F, 5e-2F, 5e-2F, // velocity (x, y, z)
        1e1F, 1e1F, 1e1F, // acceleration (x, y, z)
        1e4F, 1e4F, 1e4F, // gyro (x, y, z)
        1e2F, 1e2F, 1e2F // orientation (r, p, y)
    },
    { // Coast
        1e-2F, 1e-2F, 1e-2F, // position (x, y, z)
        1e-3F, 1e-3F, 1e-3F, // velocity (x, y, z)
        1e1F, 1e1F, 1e1F, // acceleration (x, y, z)
        1e3F, 1e3F, 1e3F, // gyro (x, y, z)
        1e1F, 1e1F, 1e1F // orientation (r, p, y)
    },
    { // Descent
        1e-1F, 1e-1F, 1e-1F, // position (x, y, z)
        1.0F, 1.0F, 1.0F, // velocity (x, y, z)
        1.0F, 1.0F, 1.0F, // acceleration (x, y, z)
        1e2F, 1e2F, 1e2F, // gyro (x, y, z)
        1e1F, 1e1F, 1e1F // orientation (r, p, y)
    },
    { // Landed
        1.0F, 1.0F, 1.0F, // position (x, y, z)
        1.0F, 1.0F, 1.0F, // velocity (x, y, z)
        1e2F, 1e2F, 1e2F, // acceleration (x, y, z)
        1e2F, 1e2F, 1e2F, // gyro (x, y, z)
        1.0F, 1.0F, 1.0F // orientation (r, p, y)
    }
};

float ukf_measurement_noise_covariance_diag[STATE_MACHINE_NUM_STATES][UKF_MEASUREMENT_DIMENSION] = {
    { // Standby
        1e2F, // pressure (pascals)
        1e-2F, 1e-2F, 1e-2F, // acceleration (x, y, z)
        1e-3F, 1e-3F, 1e-3F, // gyro angular rate (x, y, z)
        1e-2F, 1e-2F, 1e-2F // normalized magnetometer (x, y, z)
    },
    { // Motor Burn
        5.0F, // pressure (pascals)
        1e-1F, 1e-1F, 1e-1F, // acceleration (x, y, z)
        3e1F, 3e1F, 3e1F, // gyro angular rate (x, y, z)
        1e-2F, 1e-2F, 1e-2F // normalized magnetometer (x, y, z)
    },
    { // Coast
        1e3F, // pressure (pascals)
        1e-1F, 1e-1F, 1e-1F, // acceleration (x, y, z)
        1.0F, 1.0F, 1.0F, // gyro angular rate (x, y, z)
        1e-3F, 1e-3F, 1e-3F // normalized magnetometer (x, y, z)
    },
    { // Descent
        1e3F, // pressure (pascals)
        1e-1F, 1e-1F, 1e-1F, // acceleration (x, y, z)
        1e2F, 1e2F, 1e2F, // gyro angular rate (x, y, z)
        1e-1F, 1e-1F, 1e-1F // normalized magnetometer (x, y, z)
    },
    { // Landed
        1e1F, // pressure (pascals)
        1e-2F, 1e-2F, 1e-2F, // acceleration (x, y, z)
        1e1F, 1e1F, 1e1F, // gyro angular rate (x, y, z)
        1e-1F, 1e-1F, 1e-1F // normalized magnetometer (x, y, z)
    }
};

// clang-format on