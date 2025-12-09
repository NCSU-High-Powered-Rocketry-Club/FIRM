#pragma once

#include "kalman_filter_config.h"
#include "matrixhelper.h"
#include "dsp/quaternion_math_functions.h"
#include "unscented_kalman_filter.h"

#include <string.h>

void ukf_state_transition_function(const double *sigmas, double dt, int state, double *prediction);
void ukf_measurement_function(const double *sigmas, const UKF *ukfh, double *measurement_sigmas);
void calculate_initial_orientation(const double *imu_accel, const double *mag_field, double *init_quaternion, double *mag_world_frame);