#include "eskf_config.h"


// clang-format off

const float eskf_initial_state[ESKF_NOMINAL_DIM] = {
  0.0F, // altitude (z position, meters)
  0.0F, // z velocity (vertical, meters per second)
  1.0F, 0.0F, 0.0F, 0.0F, // quaternion, board->world [w, x, y, z]
};

const float eskf_initial_cov_diag[ESKF_ERROR_DIM] = {
  1e-6F, // δaltitude
  1e-6F, // δvelocity
  1e-3F, 1e-3F, 1e-3F, // δθ (angular error)
};

const float eskf_q_diag[ESKF_ERROR_DIM] = {
  1e-2F, // δaltitude
  5e-2F, // δvelocity
  1e-4F, 1e-4F, 1e-4F, // δθ
};

const float eskf_r_diag[ESKF_MEASUREMENT_DIM] = {
  1e1F, // pressure (Pa)
  1.0F, 1.0F, 1.0F, // magnetic field (normalized)
};

/* ==================================================================================
 * Sensor-to-Board Rotation Matrices — Firmware v2 Hardware (current PCB, FIRM v1.0)
 * ==================================================================================
 *
 * Board frame: +X forward, +Y left, +Z up  (right-handed)
 *
 * IMU (ICM-45686):
 *   The IMU sensor axes are rotated -45° about the board +Z axis
 *
 *               Board +X         Sensor +X
 *                   ^               ^
 *                   |                \ 
 *                   |                 \
 *  Board +Y   <-----                  /
 *                                    /
 *                                   v 
 *                               Sensor +Y
 * 
 *   R_imu_to_board = Rz(+45°):
 *       [ cos45  -sin45   0 ]     [ √2/2  -√2/2   0 ]
 *       [ sin45   cos45   0 ]  =  [ √2/2   √2/2   0 ]
 *       [   0       0     1 ]     [   0      0     1 ]
 *
 * Magnetometer (MMC5983MA):
 *   The mag sensor Z-axis points opposite to the board Z-axis, and
 *   the mag X/Y axes are rotated 90° CW (relative to board Z axis)
 *
 *     R_mag_to_board
 *       = [0  1  0]
 *         [-1 0  0]
 *         [0  0 -1]
 *
 * ==================================================================== */

const float eskf_v2_R_imu_to_board[9] = {
     SQRT2_INV, -SQRT2_INV, 0.0F,
     SQRT2_INV,  SQRT2_INV, 0.0F,
     0.0F,       0.0F,      1.0F,
};

const float eskf_v2_R_mag_to_board[9] = {
   0.0F,  1.0F,  0.0F,
  -1.0F,  0.0F,  0.0F,
   0.0F,  0.0F, -1.0F,
};

/* ==================================================================================
 * Sensor-to-Board Rotation Matrices — Firmware v1 Hardware (legacy PCB, FIRM v0.1)
 * ==================================================================================
 *
 * IMU:
 *   The IMU sensor axes are rotated +45° about the board +Z axis
 *
 *   R_imu_to_board = Rz(-45°):
 *       [ cos(-45)  -sin(-45)  0 ]     [ √2/2   √2/2   0 ]
 *       [ sin(-45)   cos(-45)  0 ]  =  [-√2/2   √2/2   0 ]
 *       [    0          0      1 ]     [   0      0     1 ]
 *
 *
 * Magnetometer:
 *   Same orientation as firmware v2.
 *
 * ==================================================================== */

const float eskf_v1_R_imu_to_board[9] = {
   SQRT2_INV,  SQRT2_INV, 0.0F,
  -SQRT2_INV,  SQRT2_INV, 0.0F,
   0.0F,       0.0F,      1.0F,
};

const float eskf_v1_R_mag_to_board[9] = {
   0.0F,  1.0F,  0.0F,
  -1.0F,  0.0F,  0.0F,
   0.0F,  0.0F, -1.0F,
};

// clang-format on