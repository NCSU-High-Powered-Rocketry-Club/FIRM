#include "error_state_kalman_filter.h"
#include "settings_manager_host.h"
#include "utest.h"

#include <math.h>

// One scenario: each phase starts from the filter state the previous phase left.
UTEST(eskf_smoke, standby_launch_landing_and_covariance) {
  host_set_firmware_version("v2.0.0");
  const float acceleration[3] = {0.0F, 0.0F, 1.0F};
  const float magnetic_field[3] = {20.0F, 5.0F, 40.0F};
  for (int i = 0; i < 400; ++i) {
    eskf_accumulate(101325.0F, acceleration, magnetic_field);
  }

  ESKF eskf;
  ASSERT_EQ(0, eskf_init(&eskf));
  const float control[6] = {0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F};
  const float measurement[4] = {101325.0F, 20.0F, 5.0F, 40.0F};
  for (int i = 0; i < 2000; ++i) {
    eskf_predict(&eskf, control, 0.005F);
    eskf_set_measurement(&eskf, measurement);
    eskf_update(&eskf);
  }

  float norm_squared = 0.0F;
  for (int i = ESKF_QUAT_W; i <= ESKF_QUAT_Z; ++i) {
    ASSERT_TRUE_MSG(isfinite(eskf.x_nom[i]), "non-finite quaternion state");
    norm_squared += eskf.x_nom[i] * eskf.x_nom[i];
  }
  ASSERT_TRUE_MSG(isfinite(eskf.x_nom[ESKF_POS_Z]), "invalid steady-state position");
  ASSERT_TRUE_MSG(isfinite(eskf.x_nom[ESKF_VEL_Z]), "invalid steady-state velocity");
  ASSERT_NEAR_MSG(1.0F, sqrtf(norm_squared), 1e-3F, "quaternion is not unit length");
  for (int row = 0; row < ESKF_ERROR_DIM; ++row) {
    ASSERT_TRUE_MSG(isfinite(eskf.P[row * ESKF_ERROR_DIM + row]), "invalid covariance diagonal");
    ASSERT_GE_MSG(eskf.P[row * ESKF_ERROR_DIM + row], 0.0F, "invalid covariance diagonal");
    for (int column = row + 1; column < ESKF_ERROR_DIM; ++column) {
      ASSERT_NEAR_MSG(eskf.P[row * ESKF_ERROR_DIM + column], eskf.P[column * ESKF_ERROR_DIM + row],
                      1e-5F, "covariance is not symmetric");
    }
  }

  /* Launch requires a sustained upward motor impulse and an independent
   * barometric rise.  A handling shock with a fixed pressure reference must
   * leave the estimator in its constrained standby phase. */
  const float motor_control[6] = {0.0F, 0.0F, 8.0F, 0.0F, 0.0F, 0.0F};
  for (int i = 0; i < 25; ++i) {
    eskf_predict(&eskf, motor_control, 0.01F);
    eskf_set_measurement(&eskf, measurement);
    eskf_update(&eskf);
  }
  ASSERT_FALSE_MSG(eskf.launched, "fixed-pressure handling shock falsely detected launch");
  eskf_predict(&eskf, control, 0.01F);
  eskf_set_measurement(&eskf, measurement);
  eskf_update(&eskf);

  const float launch_measurement[4] = {101300.0F, 20.0F, 5.0F, 40.0F};
  for (int i = 0; i < 25 && !eskf.launched; ++i) {
    eskf_predict(&eskf, motor_control, 0.01F);
    eskf_set_measurement(&eskf, launch_measurement);
    eskf_update(&eskf);
  }
  ASSERT_TRUE_MSG(eskf.launched, "confirmed motor impulse and pressure rise missed launch");

  /* After apogee, a sustained stationary IMU and pressure signal produces a
   * zero-velocity update. */
  eskf.coast_detected = 1U;
  eskf.apogee_detected = 1U;
  eskf.apogee_altitude = 10.0F;
  eskf.x_nom[ESKF_POS_Z] = 10.0F;
  eskf.x_nom[ESKF_VEL_Z] = -1.0F;
  eskf.pressure_disturbed = 0U;
  eskf.pressure_reliability = 1.0F;
  eskf.filtered_pressure_altitude = 0.0F;
  eskf.landed_stationary_time_seconds = 0.0F;
  for (int i = 0; i < 400; ++i) {
    eskf_predict(&eskf, control, 0.01F);
    eskf_set_measurement(&eskf, measurement);
    eskf_update(&eskf);
  }
  ASSERT_TRUE_MSG(eskf.landed, "stationary post-apogee state did not land");
  ASSERT_NEAR_MSG(0.0F, eskf.x_nom[ESKF_VEL_Z], 1e-6F,
                  "stationary post-apogee state did not zero velocity");

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
  eskf.launched = 1U;
  eskf.coast_detected = 1U;
  eskf.apogee_detected = 0U;
  eskf.landed = 0U;
  eskf.x_nom[ESKF_POS_Z] = 0.0F;
  eskf.x_nom[ESKF_VEL_Z] = 100.0F;
  eskf_set_measurement(&eskf, measurement);
  eskf_update(&eskf);

  const float position_velocity_determinant =
      eskf.P[ESKF_DPOS_Z * ESKF_ERROR_DIM + ESKF_DPOS_Z] *
          eskf.P[ESKF_DVEL_Z * ESKF_ERROR_DIM + ESKF_DVEL_Z] -
      eskf.P[ESKF_DPOS_Z * ESKF_ERROR_DIM + ESKF_DVEL_Z] *
          eskf.P[ESKF_DVEL_Z * ESKF_ERROR_DIM + ESKF_DPOS_Z];
  ASSERT_GE_MSG(position_velocity_determinant, -1e-6F,
                "pressure decoupling made covariance indefinite");
}

UTEST_MAIN()
