#include "eskf_config.h"
#include "eskf_functions.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int close_enough(float actual, float expected, float tolerance,
                        const char *label) {
  if (!isfinite(actual) || fabsf(actual - expected) > tolerance) {
    fprintf(stderr, "%s: expected %.9g, got %.9g\n", label, (double)expected,
            (double)actual);
    return 0;
  }
  return 1;
}

static int test_known_nominal_motion(void) {
  float identity_data[9] = {1.0F, 0.0F, 0.0F, 0.0F, 1.0F,
                            0.0F, 0.0F, 0.0F, 1.0F};
  matrix_instance_f32 identity = {3, 3, identity_data};

  float stationary_state[ESKF_NOMINAL_DIM] = {0.0F, 0.0F, 1.0F,
                                              0.0F, 0.0F, 0.0F};
  const float stationary_input[ESKF_CONTROL_DIM] = {0.0F, 0.0F, 1.0F,
                                                    0.0F, 0.0F, 0.0F};
  eskf_nominal_predict(stationary_state, stationary_input, 0.1F, &identity);
  if (!close_enough(stationary_state[ESKF_POS_Z], 0.0F, 1e-6F,
                    "stationary position") ||
      !close_enough(stationary_state[ESKF_VEL_Z], 0.0F, 1e-6F,
                    "stationary velocity")) {
    return 0;
  }

  float rotation_state[ESKF_NOMINAL_DIM] = {0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F};
  const float rotation_input[ESKF_CONTROL_DIM] = {0.0F, 0.0F, 1.0F,
                                                  0.0F, 0.0F, 90.0F};
  eskf_nominal_predict(rotation_state, rotation_input, 1.0F, &identity);
  return close_enough(rotation_state[ESKF_QUAT_W], SQRT2_INV, 1e-5F,
                      "yaw quaternion w") &&
         close_enough(rotation_state[ESKF_QUAT_Z], SQRT2_INV, 1e-5F,
                      "yaw quaternion z");
}

static int test_pressure_measurement_jacobian(void) {
  const float initial_pressure = 101325.0F;
  const float mag_world[3] = {0.2F, 0.4F, 0.8F};
  const float identity[9] = {1.0F, 0.0F, 0.0F, 0.0F, 1.0F,
                             0.0F, 0.0F, 0.0F, 1.0F};
  matrix_instance_f32 rotation = {3, 3, (float *)identity};
  float state[ESKF_NOMINAL_DIM] = {1250.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F};
  float jacobian[ESKF_MEASUREMENT_DIM * ESKF_ERROR_DIM];
  eskf_measurement_jacobian(state, initial_pressure, mag_world, identity,
                            jacobian);

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
  const float numerical =
      (upper_measurement[0] - lower_measurement[0]) / (2.0F * epsilon);
  return close_enough(jacobian[0], numerical, 0.02F,
                      "pressure measurement Jacobian");
}

static int test_velocity_error_jacobian(void) {
  float identity_data[9] = {1.0F, 0.0F, 0.0F, 0.0F, 1.0F,
                            0.0F, 0.0F, 0.0F, 1.0F};
  matrix_instance_f32 identity = {3, 3, identity_data};
  const float orientation[3] = {0.4F, -0.3F, 0.7F};
  const float input[ESKF_CONTROL_DIM] = {0.3F, -0.7F, 1.2F, 0.0F, 0.0F, 0.0F};
  const float dt = 0.02F;
  const float epsilon = 1e-3F;
  float state[ESKF_NOMINAL_DIM] = {0.0F};
  rotvec_to_quat(orientation, &state[ESKF_QUAT_W]);

  float jacobian[ESKF_ERROR_DIM * ESKF_ERROR_DIM];
  eskf_error_jacobian(state, input, dt, &identity, jacobian);

  for (int axis = 0; axis < 3; ++axis) {
    float positive_state[ESKF_NOMINAL_DIM];
    float negative_state[ESKF_NOMINAL_DIM];
    memcpy(positive_state, state, sizeof(state));
    memcpy(negative_state, state, sizeof(state));

    float positive_error[3] = {0.0F};
    float negative_error[3] = {0.0F};
    positive_error[axis] = epsilon;
    negative_error[axis] = -epsilon;
    float positive_delta_q[4], negative_delta_q[4];
    float positive_q[4], negative_q[4];
    rotvec_to_quat(positive_error, positive_delta_q);
    rotvec_to_quat(negative_error, negative_delta_q);
    quaternion_product_f32(&state[ESKF_QUAT_W], positive_delta_q, positive_q);
    quaternion_product_f32(&state[ESKF_QUAT_W], negative_delta_q, negative_q);
    memcpy(&positive_state[ESKF_QUAT_W], positive_q, sizeof(positive_q));
    memcpy(&negative_state[ESKF_QUAT_W], negative_q, sizeof(negative_q));

    eskf_nominal_predict(positive_state, input, dt, &identity);
    eskf_nominal_predict(negative_state, input, dt, &identity);
    const float numerical =
        (positive_state[ESKF_VEL_Z] - negative_state[ESKF_VEL_Z]) /
        (2.0F * epsilon);
    if (!close_enough(
            jacobian[ESKF_DVEL_Z * ESKF_ERROR_DIM + ESKF_DTHETA_X + axis],
            numerical, 2e-4F, "velocity error Jacobian")) {
      return 0;
    }
  }
  return 1;
}

static int test_magnetometer_measurement_jacobian(void) {
  const float mag_world[3] = {0.2F, -0.5F, 0.84F};
  const float orientation[3] = {-0.5F, 0.2F, 0.6F};
  const float epsilon = 1e-3F;
  float state[ESKF_NOMINAL_DIM] = {125.0F, 0.0F};
  rotvec_to_quat(orientation, &state[ESKF_QUAT_W]);
  float rotation_data[9];
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      rotation_data[row * 3 + column] = eskf_v2_R_mag_to_board[column * 3 + row];
    }
  }
  matrix_instance_f32 rotation = {3, 3, rotation_data};

  float jacobian[ESKF_MEASUREMENT_DIM * ESKF_ERROR_DIM];
  eskf_measurement_jacobian(state, 101325.0F, mag_world, rotation_data,
                            jacobian);

  for (int axis = 0; axis < 3; ++axis) {
    float positive_state[ESKF_NOMINAL_DIM];
    float negative_state[ESKF_NOMINAL_DIM];
    memcpy(positive_state, state, sizeof(state));
    memcpy(negative_state, state, sizeof(state));

    float positive_error[3] = {0.0F};
    float negative_error[3] = {0.0F};
    positive_error[axis] = epsilon;
    negative_error[axis] = -epsilon;
    float positive_delta_q[4], negative_delta_q[4];
    float positive_q[4], negative_q[4];
    rotvec_to_quat(positive_error, positive_delta_q);
    rotvec_to_quat(negative_error, negative_delta_q);
    quaternion_product_f32(&state[ESKF_QUAT_W], positive_delta_q, positive_q);
    quaternion_product_f32(&state[ESKF_QUAT_W], negative_delta_q, negative_q);
    memcpy(&positive_state[ESKF_QUAT_W], positive_q, sizeof(positive_q));
    memcpy(&negative_state[ESKF_QUAT_W], negative_q, sizeof(negative_q));

    float positive_measurement[ESKF_MEASUREMENT_DIM];
    float negative_measurement[ESKF_MEASUREMENT_DIM];
    eskf_measurement_function(positive_state, 101325.0F, mag_world, &rotation,
                              positive_measurement);
    eskf_measurement_function(negative_state, 101325.0F, mag_world, &rotation,
                              negative_measurement);
    for (int component = 0; component < 3; ++component) {
      const float numerical = (positive_measurement[1 + component] -
                               negative_measurement[1 + component]) /
                              (2.0F * epsilon);
      if (!close_enough(
              jacobian[(1 + component) * ESKF_ERROR_DIM + ESKF_DTHETA_X + axis],
              numerical, 2e-4F, "magnetometer measurement Jacobian")) {
        return 0;
      }
    }
  }
  return 1;
}

int main(void) {
  if (!test_known_nominal_motion() || !test_pressure_measurement_jacobian() ||
      !test_velocity_error_jacobian() ||
      !test_magnetometer_measurement_jacobian()) {
    return 1;
  }
  return 0;
}
