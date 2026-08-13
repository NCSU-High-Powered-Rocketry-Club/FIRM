#include "error_state_kalman_filter.h"
#include "settings_manager_host.h"

#include <math.h>
#include <stdio.h>

int main(void) {
  host_set_firmware_version("v2.0.0");
  const float acceleration[3] = {0.0F, 0.0F, 1.0F};
  const float magnetic_field[3] = {20.0F, 5.0F, 40.0F};
  for (int i = 0; i < 400; ++i) {
    eskf_accumulate(101325.0F, acceleration, magnetic_field);
  }

  ESKF eskf;
  if (eskf_init(&eskf) != 0) {
    return 1;
  }
  const float control[6] = {0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F};
  const float measurement[4] = {101325.0F, 20.0F, 5.0F, 40.0F};
  for (int i = 0; i < 2000; ++i) {
    eskf_predict(&eskf, control, 0.005F);
    eskf_set_measurement(&eskf, measurement);
    eskf_update(&eskf);
  }

  float norm_squared = 0.0F;
  for (int i = ESKF_QUAT_W; i <= ESKF_QUAT_Z; ++i) {
    if (!isfinite(eskf.x_nom[i])) {
      fprintf(stderr, "non-finite quaternion state\n");
      return 1;
    }
    norm_squared += eskf.x_nom[i] * eskf.x_nom[i];
  }
  if (!isfinite(eskf.x_nom[ESKF_POS_Z]) || !isfinite(eskf.x_nom[ESKF_VEL_Z]) ||
      fabsf(sqrtf(norm_squared) - 1.0F) > 1e-3F) {
    fprintf(stderr, "invalid steady-state ESKF result\n");
    return 1;
  }
  for (int row = 0; row < ESKF_ERROR_DIM; ++row) {
    if (!isfinite(eskf.P[row * ESKF_ERROR_DIM + row]) ||
        eskf.P[row * ESKF_ERROR_DIM + row] < 0.0F) {
      fprintf(stderr, "invalid covariance diagonal\n");
      return 1;
    }
    for (int column = row + 1; column < ESKF_ERROR_DIM; ++column) {
      const float difference = fabsf(eskf.P[row * ESKF_ERROR_DIM + column] -
                                     eskf.P[column * ESKF_ERROR_DIM + row]);
      if (difference > 1e-5F) {
        fprintf(stderr, "covariance is not symmetric\n");
        return 1;
      }
    }
  }

  /* A selectively decoupled pressure gain is no longer the optimal Kalman
   * gain.  Its covariance update must still preserve positive semidefiniteness.
   */
  for (int i = 0; i < ESKF_ERROR_DIM * ESKF_ERROR_DIM; ++i) {
    eskf.P[i] = 0.0F;
  }
  eskf.P[ESKF_DPOS_Z * ESKF_ERROR_DIM + ESKF_DPOS_Z] = 1.0F;
  eskf.P[ESKF_DPOS_Z * ESKF_ERROR_DIM + ESKF_DVEL_Z] = 0.99F;
  eskf.P[ESKF_DVEL_Z * ESKF_ERROR_DIM + ESKF_DPOS_Z] = 0.99F;
  eskf.P[ESKF_DVEL_Z * ESKF_ERROR_DIM + ESKF_DVEL_Z] = 1.0F;
  for (int i = ESKF_DTHETA_X; i <= ESKF_DTHETA_Z; ++i) {
    eskf.P[i * ESKF_ERROR_DIM + i] = 1e-3F;
  }
  eskf.x_nom[ESKF_POS_Z] = 0.0F;
  eskf.x_nom[ESKF_VEL_Z] = 100.0F;
  eskf_set_measurement(&eskf, measurement);
  eskf_update(&eskf);

  const float position_velocity_determinant =
      eskf.P[ESKF_DPOS_Z * ESKF_ERROR_DIM + ESKF_DPOS_Z] *
          eskf.P[ESKF_DVEL_Z * ESKF_ERROR_DIM + ESKF_DVEL_Z] -
      eskf.P[ESKF_DPOS_Z * ESKF_ERROR_DIM + ESKF_DVEL_Z] *
          eskf.P[ESKF_DVEL_Z * ESKF_ERROR_DIM + ESKF_DPOS_Z];
  if (position_velocity_determinant < -1e-6F) {
    fprintf(stderr, "pressure decoupling made covariance indefinite\n");
    return 1;
  }
  return 0;
}
