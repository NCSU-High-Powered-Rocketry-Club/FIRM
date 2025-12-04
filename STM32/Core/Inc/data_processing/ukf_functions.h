#pragma once

#include "kalman_filter_config.h"
#include <string.h>
#include "matrixhelper.h"

void ukf_state_transition_function(const double *sigmas, double dt, int state, double *prediction);