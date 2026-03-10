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

const float eskf_q_diag[ESKF_NUM_FLIGHT_STATES][ESKF_ERROR_DIM] = {
    { /* ESKF_STATE_INIT */
        1e-5F, 1e-5F, 1e-5F,  /* δposition  (clamped to zero) */
        1e-5F, 1e-5F, 1e-5F,  /* δvelocity  (clamped to zero) */
        1e-4F, 1e-4F, 1e-4F,  /* δθ */
    },
    { /* ESKF_STATE_RUNNING */
        1e-1F, 1e-1F, 1e-2F,  /* δposition */
        1e-1F, 1e-1F, 1e-2F,  /* δvelocity */
        1e-3F, 1e-3F, 1e-3F,  /* δθ */
    },
};

const float eskf_r_diag[ESKF_NUM_FLIGHT_STATES][ESKF_MEASUREMENT_DIM] = {
    { /* ESKF_STATE_INIT */
        5e1F,                  /* pressure */
        1e-2F, 1e-2F, 1e-2F,  /* mag (normalised) */
    },
    { /* ESKF_STATE_RUNNING */
        5e1F,                  /* pressure */
        1e-3F, 1e-3F, 1e-3F,  /* mag (normalised) */
    },
};

/* IMU sensor → vehicle rotation (45° CCW about Z), row-major 3×3 */
const float R_IMU_TO_VEHICLE[9] = {
     SQRT2_INV, -SQRT2_INV, 0.0F,
     SQRT2_INV,  SQRT2_INV, 0.0F,
     0.0F,       0.0F,      1.0F,
};

/* Vehicle → magnetometer sensor: z-axis flipped, row-major 3×3 */
const float R_VEHICLE_TO_MAG[9] = {
    1.0F, 0.0F,  0.0F,
    0.0F, 1.0F,  0.0F,
    0.0F, 0.0F, -1.0F,
};

// clang-format on