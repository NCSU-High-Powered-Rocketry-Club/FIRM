#include "kalman_filter_config.h"

/* ====================================================================
 * Sensor rotation matrices
 * ====================================================================
 * Firmware v2.x.x hardware (current):
 *   IMU: sensor mounted -45° about board +Z → apply Rz(+45°) to get board from sensor
 *
 *     R_imu_to_board (Rz +45°):
 *       [ √2/2  -√2/2   0 ]
 *       [ √2/2   √2/2   0 ]
 *       [   0      0    1 ]
 *
 *   Mag: sensor axes map to board as flip Z then Rz(+90°) → R_mag_to_board:
 *       [  0   1   0 ]
 *       [ -1   0   0 ]
 *       [  0   0  -1 ]
 *
 * Firmware v1.x.x hardware (legacy):
 *   IMU: sensor mounted +45° about board +Z → apply Rz(-45°)
 *
 *     R_imu_to_board (Rz -45°):
 *       [ √2/2   √2/2   0 ]
 *       [-√2/2   √2/2   0 ]
 *       [   0      0    1 ]
 *
 *   Mag: same as v2
 * ==================================================================== */

// clang-format off
const float ukf_v2_R_imu_to_board[9] = {
     SQRT2_INV, -SQRT2_INV, 0.0F,
     SQRT2_INV,  SQRT2_INV, 0.0F,
     0.0F,       0.0F,      1.0F,
};

const float ukf_v2_R_mag_to_board[9] = {
     0.0F,  1.0F,  0.0F,
    -1.0F,  0.0F,  0.0F,
     0.0F,  0.0F, -1.0F,
};

const float ukf_v1_R_imu_to_board[9] = {
     SQRT2_INV,  SQRT2_INV, 0.0F,
    -SQRT2_INV,  SQRT2_INV, 0.0F,
     0.0F,       0.0F,      1.0F,
};

const float ukf_v1_R_mag_to_board[9] = {
     0.0F,  1.0F,  0.0F,
    -1.0F,  0.0F,  0.0F,
     0.0F,  0.0F, -1.0F,
};
// clang-format on

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
        1e-5F, 1e-5F, 1e-5F, // position (x, y, z)
        1e-5F, 1e-5F, 1e-5F, // velocity (x, y, z)
        1e-3F, 1e-3F, 1e-3F, // acceleration (x, y, z)
        1.0F, 1.0F, 1.0F, // gyro (x, y, z)
        1e-1F, 1e-1F, 1e-1F // orientation (r, p, y)
    },
    { // Motor Burn
        1.0F, 1.0F, 1e-3F, // position (x, y, z)
        1e-1F, 1e-1F, 1e-3F, // velocity (x, y, z)
        1.0F, 1.0F, 1e3F, // acceleration (x, y, z)
        1.0F, 1.0F, 1.0F, // gyro (x, y, z)
        1.0F, 1.0F, 1.0F // orientation (r, p, y)
    },
    { // Coast
        1e-1F, 1e-1F, 1e-1F, // position (x, y, z)
        1e-1F, 1e-1F, 1e-3F, // velocity (x, y, z)
        1e-1F, 1e-1F, 1e-1F, // acceleration (x, y, z)
        1.0F, 1.0F, 1.0F, // gyro (x, y, z)
        1.0F, 1.0F, 1.0F // orientation (r, p, y)
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
        1e1F, // pressure (pascals)
        1e-2F, 1e-2F, 1e-2F, // acceleration (x, y, z)
        1e-3F, 1e-3F, 1e-3F, // gyro angular rate (x, y, z)
        1e-2F, 1e-2F, 1e-2F // normalized magnetometer (x, y, z)
    },
    { // Motor Burn
        1e1F, // pressure (pascals)
        5e-2F, 5e-2F, 5e-2F, // acceleration (x, y, z)
        1.0F, 1.0F, 1.0F, // gyro angular rate (x, y, z)
        1e-2F, 1e-2F, 1e-2F // normalized magnetometer (x, y, z)
    },
    { // Coast
        1e1F, // pressure (pascals)
        1e-3F, 1e-3F, 1e-3F, // acceleration (x, y, z)
        1.0F, 1.0F, 1.0F, // gyro angular rate (x, y, z)
        1e-1F, 1e-1F, 1e-1F // normalized magnetometer (x, y, z)
    },
    { // Descent
        1e1F, // pressure (pascals)
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