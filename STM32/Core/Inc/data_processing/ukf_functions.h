#pragma once

#include <arm_math.h>
#include "kalman_filter_config.h"
#include "matrixhelper.h"
#include "dsp/quaternion_math_functions.h"
#include "unscented_kalman_filter.h"


#include <string.h>

void ukf_state_transition_function(const double *sigmas, double dt, State state, double *prediction);
void ukf_measurement_function(const double *sigmas, const UKF *ukfh, double *measurement_sigmas);
