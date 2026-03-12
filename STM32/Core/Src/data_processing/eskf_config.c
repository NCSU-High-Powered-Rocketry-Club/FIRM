#include "eskf_config.h"

/* ====================================================================
 * ESKF Tuning Arrays
 * Ported from Python UKF/constants.py — ESKF section
 * ==================================================================== */

// clang-format off

const float eskf_initial_state[ESKF_NOMINAL_DIM] = {
  0.0F, 0.0F, 0.0F,          /* position  (x, y, z) */
  0.0F, 0.0F, 0.0F,          /* velocity  (x, y, z) */
  1.0F, 0.0F, 0.0F, 0.0F,   /* quaternion (w, x, y, z) — identity */
};

const float eskf_initial_cov_diag[ESKF_ERROR_DIM] = {
  1e-6F, 1e-6F, 1e-6F,      /* δposition */
  1e-6F, 1e-6F, 1e-6F,      /* δvelocity */
  1e-3F, 1e-3F, 1e-3F,      /* δθ (angular error) */
};

const float eskf_q_diag[ESKF_ERROR_DIM] = {
  1e-1F, 1e-1F, 1e-2F,  /* δposition */
  1e-1F, 1e-1F, 1e-2F,  /* δvelocity */
  1e-3F, 1e-3F, 1e-3F,  /* δθ */
};

const float eskf_r_diag[ESKF_MEASUREMENT_DIM] = {
  5e1F,                  /* pressure */
  1e-3F, 1e-3F, 1e-3F,  /* mag (normalised) */
};

/* ====================================================================
 * Sensor-to-Board Rotation Matrices — v2 Hardware (current PCB)
 * ====================================================================
 *
 * Board frame: +X forward, +Y left, +Z up  (right-handed)
 *
 * IMU (ICM-45686):
 *   The IMU sensor axes are rotated -45° about the board +Z axis
 *   (right-hand rule: sensor +X points between board +X and board -Y).
 *   To transform sensor readings into the board frame, apply Rz(+45°):
 *
 *       Board +Y              Sensor +Y
 *           ^                    ^
 *           |                     \ 45°
 *           |                      \
 *   Board +X ---->        Sensor +X ---->
 *
 *   R_imu_to_board = Rz(+45°):
 *       [ cos45  -sin45   0 ]     [ √2/2  -√2/2   0 ]
 *       [ sin45   cos45   0 ]  =  [ √2/2   √2/2   0 ]
 *       [   0       0     1 ]     [   0      0     1 ]
 *
 *   board_x =  √2/2 · sensor_x  −  √2/2 · sensor_y
 *   board_y =  √2/2 · sensor_x  +  √2/2 · sensor_y
 *   board_z =  sensor_z
 *
 * Magnetometer (MMC5983MA):
 *   The mag sensor Z-axis points opposite to the board Z-axis, and
 *   the mag X/Y axes are rotated 90° CW (about the board +Z axis)
 *   relative to the board axes.
 *
 *   Sensor-to-board transform: first flip Z, then Rz(+90°)
 *     R_mag_to_board = Rz(90°) @ Fz
 *       = [0 -1  0]   [1  0  0]   [0 -1  0]
 *         [1  0  0] @ [0  1  0] = [1  0  0]
 *         [0  0  1]   [0  0 -1]   [0  0 -1]
 *
 *   Board-to-sensor (stored as R_board_to_mag) = transpose:
 *       [ 0   1   0 ]
 *       [-1   0   0 ]
 *       [ 0   0  -1 ]
 *
 * ==================================================================== */

const float eskf_v2_R_imu_to_board[9] = {
     SQRT2_INV, -SQRT2_INV, 0.0F,
     SQRT2_INV,  SQRT2_INV, 0.0F,
     0.0F,       0.0F,      1.0F,
};

const float eskf_v2_R_board_to_mag[9] = {
   0.0F,  1.0F,  0.0F,
  -1.0F,  0.0F,  0.0F,
   0.0F,  0.0F, -1.0F,
};

/* ====================================================================
 * Sensor-to-Board Rotation Matrices — v1 Hardware (legacy PCB)
 * ====================================================================
 *
 * IMU:
 *   The v1 IMU sensor axes are rotated +45° about the board +Z axis
 *   (sensor +X points between board +X and board +Y).
 *   To transform sensor readings into the board frame, apply Rz(-45°):
 *
 *   R_imu_to_board = Rz(-45°):
 *       [ cos(-45)  -sin(-45)  0 ]     [ √2/2   √2/2   0 ]
 *       [ sin(-45)   cos(-45)  0 ]  =  [-√2/2   √2/2   0 ]
 *       [    0          0      1 ]     [   0      0     1 ]
 *
 *   board_x =  √2/2 · sensor_x  +  √2/2 · sensor_y
 *   board_y = -√2/2 · sensor_x  +  √2/2 · sensor_y
 *   board_z =  sensor_z
 *
 * Magnetometer:
 *   Same orientation as v2 hardware (flip Z, then Rz(+90°)).
 *
 * ==================================================================== */

const float eskf_v1_R_imu_to_board[9] = {
   SQRT2_INV,  SQRT2_INV, 0.0F,
  -SQRT2_INV,  SQRT2_INV, 0.0F,
   0.0F,       0.0F,      1.0F,
};

const float eskf_v1_R_board_to_mag[9] = {
   0.0F,  1.0F,  0.0F,
  -1.0F,  0.0F,  0.0F,
   0.0F,  0.0F, -1.0F,
};

// clang-format on