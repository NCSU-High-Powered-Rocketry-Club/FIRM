#include "eskf_config.h"
#include "eskf_functions.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int close_enough(float actual, float expected, float tolerance, const char *label) {
  if (!isfinite(actual) || fabsf(actual - expected) > tolerance) {
    fprintf(stderr, "%s: expected %.9g, got %.9g\n", label, (double)expected, (double)actual);
    return 0;
  }
  return 1;
}

static int test_known_nominal_motion(void) {
  float identity_data[9] = {1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F};
  matrix_instance_f32 identity = {3, 3, identity_data};

  float stationary_state[ESKF_NOMINAL_DIM] = {0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F};
  const float stationary_input[ESKF_CONTROL_DIM] = {0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F};
  eskf_nominal_predict(stationary_state, stationary_input, 0.1F, &identity);
  if (!close_enough(stationary_state[ESKF_POS_Z], 0.0F, 1e-6F, "stationary position") ||
      !close_enough(stationary_state[ESKF_VEL_Z], 0.0F, 1e-6F, "stationary velocity")) {
    return 0;
  }

  float rotation_state[ESKF_NOMINAL_DIM] = {0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F};
  const float rotation_input[ESKF_CONTROL_DIM] = {0.0F, 0.0F, 1.0F,
                                                   0.0F, 0.0F, 90.0F};
  eskf_nominal_predict(rotation_state, rotation_input, 1.0F, &identity);
  return close_enough(rotation_state[ESKF_QUAT_W], SQRT2_INV, 1e-5F, "yaw quaternion w") &&
         close_enough(rotation_state[ESKF_QUAT_Z], SQRT2_INV, 1e-5F, "yaw quaternion z");
}

static int test_pressure_measurement_jacobian(void) {
  const float initial_pressure = 101325.0F;
  const float mag_world[3] = {0.2F, 0.4F, 0.8F};
  const float identity[9] = {1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F};
  matrix_instance_f32 rotation = {3, 3, (float *)identity};
  float state[ESKF_NOMINAL_DIM] = {1250.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F};
  float jacobian[ESKF_MEASUREMENT_DIM * ESKF_ERROR_DIM];
  eskf_measurement_jacobian(state, initial_pressure, mag_world, identity, jacobian);

  const float epsilon = 0.25F;
  float lower_state[ESKF_NOMINAL_DIM];
  float upper_state[ESKF_NOMINAL_DIM];
  memcpy(lower_state, state, sizeof(state));
  memcpy(upper_state, state, sizeof(state));
  lower_state[ESKF_POS_Z] -= epsilon;
  upper_state[ESKF_POS_Z] += epsilon;
  float lower_measurement[ESKF_MEASUREMENT_DIM];
  float upper_measurement[ESKF_MEASUREMENT_DIM];
  eskf_measurement_function(lower_state, initial_pressure, mag_world, &rotation,
                            lower_measurement);
  eskf_measurement_function(upper_state, initial_pressure, mag_world, &rotation,
                            upper_measurement);
  const float numerical = (upper_measurement[0] - lower_measurement[0]) / (2.0F * epsilon);
  return close_enough(jacobian[0], numerical, 0.02F, "pressure measurement Jacobian");
}

int main(void) {
  if (!test_known_nominal_motion() || !test_pressure_measurement_jacobian()) {
    return 1;
  }
  return 0;
}
