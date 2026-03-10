#include "state_machine.h"
#include "error_state_kalman_filter.h"
#include <math.h>

/* ------------------------------------------------------------------
 * Helper: load Q and R diagonal values for the current flight state
 * ------------------------------------------------------------------ */
static void set_state_matrices(ESKF *eskf) {
  ESKFFlightState s = eskf->flight_state;
  for (int i = 0; i < ESKF_ERROR_DIM; i++) {
    eskf->Q[i * ESKF_ERROR_DIM + i] = eskf_q_diag[s][i];
  }
  for (int i = 0; i < ESKF_MEASUREMENT_DIM; i++) {
    eskf->R[i * ESKF_MEASUREMENT_DIM + i] = eskf_r_diag[s][i];
  }
}

void eskf_state_init(ESKF *eskf) {
  eskf->flight_state = ESKF_STATE_INIT;
  memset(eskf->Q, 0, sizeof(float) * ESKF_ERROR_DIM * ESKF_ERROR_DIM);
  memset(eskf->R, 0, sizeof(float) * ESKF_MEASUREMENT_DIM * ESKF_MEASUREMENT_DIM);
  set_state_matrices(eskf);
}

void eskf_state_update(ESKF *eskf) {
  if (eskf->flight_state != ESKF_STATE_INIT) {
    return;  /* no further transitions */
  }
  if (eskf->elapsed_time < ESKF_INIT_DURATION_S) {
    return;
  }

  /* ---- compute fixed IMU biases from accumulated samples ---------- */
  if (eskf->accum_count > 0) {
    float inv_n = 1.0F / (float)eskf->accum_count;
    float mean_accel[3], mean_gyro[3];
    for (int i = 0; i < 3; i++) {
      mean_accel[i] = eskf->accel_accum[i] * inv_n;
      mean_gyro[i]  = eskf->gyro_accum[i]  * inv_n;
    }

    /* expected gravity in sensor frame (g-units):
     *   gravity_vehicle = R_v2w^T @ [0, 0, GRAVITY]  → in vehicle frame (m/s²)
     *   gravity_sensor  = R_IMU_TO_VEHICLE^T @ (gravity_vehicle / GRAVITY)  → g-units
     *
     * We need the current orientation quaternion to do this. */
    float quat[4];
    memcpy(quat, &eskf->x_nom[ESKF_QUAT_W], sizeof(float) * 4);
    quaternion_normalize_f32(quat);

    /* R_v2w from quaternion */
    float R_v2w_data[9];
    arm_matrix_instance_f32 R_v2w = {3, 3, R_v2w_data};
    quat_to_rotation_matrix_f32(quat, &R_v2w);

    /* gravity_vehicle = R_v2w^T @ [0, 0, g]  (row 2 of R_v2w, i.e. indices 6,7,8) */
    float gravity_vehicle[3] = {
        R_v2w_data[6] * GRAVITY_METERS_PER_SECOND_SQUARED,
        R_v2w_data[7] * GRAVITY_METERS_PER_SECOND_SQUARED,
        R_v2w_data[8] * GRAVITY_METERS_PER_SECOND_SQUARED,
    };

    /* gravity_sensor = R_IMU_TO_VEHICLE^T @ (gravity_vehicle / GRAVITY)
     * R_IMU_TO_VEHICLE^T rows = columns of R_IMU_TO_VEHICLE (row-major) */
    float gv_g[3] = {
        gravity_vehicle[0] / GRAVITY_METERS_PER_SECOND_SQUARED,
        gravity_vehicle[1] / GRAVITY_METERS_PER_SECOND_SQUARED,
        gravity_vehicle[2] / GRAVITY_METERS_PER_SECOND_SQUARED,
    };
    float gravity_sensor[3];
    /* R_IMU_TO_VEHICLE^T[row][col] = R_IMU_TO_VEHICLE[col][row] */
    for (int r = 0; r < 3; r++) {
      gravity_sensor[r] = 0.0F;
      for (int c = 0; c < 3; c++) {
        gravity_sensor[r] += R_IMU_TO_VEHICLE[c * 3 + r] * gv_g[c];
      }
    }

    /* accel_bias = mean_accel − expected_gravity  (sensor frame, g-units) */
    for (int i = 0; i < 3; i++) {
      eskf->accel_bias[i] = mean_accel[i] - gravity_sensor[i];
      eskf->gyro_bias[i]  = mean_gyro[i];
    }
  }

  /* ---- transition ------------------------------------------------- */
  eskf->flight_state = ESKF_STATE_RUNNING;
  set_state_matrices(eskf);
}