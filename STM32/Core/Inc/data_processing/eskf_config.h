#pragma once
#include <stdint.h>

/* ====================================================================
 * ESKF (Error-State Extended Kalman Filter) Configuration
 * ====================================================================
 * Ported from the Python ESKF implementation.
 *
 * Nominal state (10): pos(3), vel(3), quat(4)   [w,x,y,z]
 * Error   state  (9): δpos(3), δvel(3), δθ(3)
 * Measurement    (4): pressure(1), mag_sensor(3) (normalised)
 * Control        (6): accel(3) + gyro(3)         (sensor frame, g / deg·s⁻¹)
 * ==================================================================== */

/* ---- dimensions --------------------------------------------------- */
#define ESKF_NOMINAL_DIM        10
#define ESKF_ERROR_DIM           9
#define ESKF_MEASUREMENT_DIM     4
#define ESKF_CONTROL_DIM         6

/* ---- flight state enum -------------------------------------------- */
typedef enum {
  ESKF_STATE_INIT    = 0,
  ESKF_STATE_RUNNING = 1,
} ESKFFlightState;

#define ESKF_NUM_FLIGHT_STATES   2

/* ---- nominal-state index enum ------------------------------------- */
typedef enum {
  ESKF_POS_X  = 0, ESKF_POS_Y  = 1, ESKF_POS_Z  = 2,
  ESKF_VEL_X  = 3, ESKF_VEL_Y  = 4, ESKF_VEL_Z  = 5,
  ESKF_QUAT_W = 6, ESKF_QUAT_X = 7, ESKF_QUAT_Y = 8, ESKF_QUAT_Z = 9,
} ESKFNominalIndex;

/* ---- error-state index enum --------------------------------------- */
typedef enum {
  ESKF_DPOS_X   = 0, ESKF_DPOS_Y   = 1, ESKF_DPOS_Z   = 2,
  ESKF_DVEL_X   = 3, ESKF_DVEL_Y   = 4, ESKF_DVEL_Z   = 5,
  ESKF_DTHETA_X = 6, ESKF_DTHETA_Y = 7, ESKF_DTHETA_Z = 8,
} ESKFErrorIndex;

/* ---- physics ------------------------------------------------------ */
#define GRAVITY_METERS_PER_SECOND_SQUARED  9.798F
#define PRESSURE_ALTITUDE_CONST            44330.0F
#define PRESSURE_EXPONENT                  5.255876F

/* ---- IMU sensor-to-vehicle rotation (45° CCW about Z) ------------- */
#define SQRT2_INV  0.70710678118F

/* ---- pressure→velocity decoupling sigmoid ------------------------- */
#define ESKF_PV_COUPLING_SPEED       20.0F
#define ESKF_PV_COUPLING_SHARPNESS    1.0F

/* ---- init phase --------------------------------------------------- */
#define ESKF_INIT_DURATION_S          0.5F

/* ---- tuning arrays (defined in kalman_filter_config.c) ------------ */
extern const float eskf_initial_state[ESKF_NOMINAL_DIM];
extern const float eskf_initial_cov_diag[ESKF_ERROR_DIM];
extern const float eskf_q_diag[ESKF_NUM_FLIGHT_STATES][ESKF_ERROR_DIM];
extern const float eskf_r_diag[ESKF_NUM_FLIGHT_STATES][ESKF_MEASUREMENT_DIM];

/* ---- static IMU rotation matrices (defined in kalman_filter_config.c) */
extern const float R_IMU_TO_VEHICLE[9];       /* 3×3 row-major */
extern const float R_VEHICLE_TO_MAG[9];       /* 3×3 row-major */

