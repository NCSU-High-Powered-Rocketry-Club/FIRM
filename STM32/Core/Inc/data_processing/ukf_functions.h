#pragma once

#include "matrixhelper.h"
#include "kalman_filter_config.h"
#include "unscented_kalman_filter.h"


#include <string.h>

void ukf_state_transition_function(const float *sigmas, float dt, State state, float *prediction);
void ukf_measurement_function(const float *sigmas, const UKF *ukfh, float *measurement_sigmas);
