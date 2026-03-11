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
 * Sensor Rotation Matrices — v2 Hardware (current PCB revision)
 * ====================================================================
 *
 * IMU (ICM-45686) is mounted rotated 45° clockwise from the vehicle
 * X-axis when viewed from above.  To convert sensor readings into the
 * vehicle body frame we apply a 45° counter-clockwise rotation about Z:
 *
 *        Vehicle +Y                Sensor +Y
 *            ^                        ^  /
 *            |                        | /  45°
 *            |                        |/
 *   Vehicle +X ---->        Sensor +X ---->
 *
 *   R_imu_to_vehicle = Rz(+45°) =
 *       [ cos45  -sin45   0 ]     [ √2/2  -√2/2   0 ]
 *       [ sin45   cos45   0 ]  =  [ √2/2   √2/2   0 ]
 *       [   0       0     1 ]     [   0      0     1 ]
 *
 * Magnetometer (MMC5983MA) is mounted with its Z-axis pointing
 * opposite to the vehicle Z-axis.  X and Y are aligned:
 *
 *   R_vehicle_to_mag =
 *       [ 1   0   0 ]
 *       [ 0   1   0 ]
 *       [ 0   0  -1 ]
 *
 * ==================================================================== */

const float eskf_v2_R_imu_to_vehicle[9] = {
     SQRT2_INV, -SQRT2_INV, 0.0F,
     SQRT2_INV,  SQRT2_INV, 0.0F,
     0.0F,       0.0F,      1.0F,
};

const float eskf_v2_R_vehicle_to_mag[9] = {
  1.0F, 0.0F,  0.0F,
  0.0F, 1.0F,  0.0F,
  0.0F, 0.0F, -1.0F,
};

/* ====================================================================
 * Sensor Rotation Matrices — v1 Hardware (legacy PCB revision)
 * ====================================================================
 *
 * TODO: Fill in the actual sensor orientations for v1 hardware.
 *       Use the same convention as v2 above:
 *       - R_imu_to_vehicle: rotation from IMU sensor frame → vehicle body frame
 *       - R_vehicle_to_mag: rotation from vehicle body frame → magnetometer sensor frame
 *
 * ==================================================================== */

const float eskf_v1_R_imu_to_vehicle[9] = {
  /* TODO: fill in v1 IMU orientation */
  1.0F, 0.0F, 0.0F,
  0.0F, 1.0F, 0.0F,
  0.0F, 0.0F, 1.0F,
};

const float eskf_v1_R_vehicle_to_mag[9] = {
  /* TODO: fill in v1 magnetometer orientation */
  1.0F, 0.0F, 0.0F,
  0.0F, 1.0F, 0.0F,
  0.0F, 0.0F, 1.0F,
};

// clang-format on