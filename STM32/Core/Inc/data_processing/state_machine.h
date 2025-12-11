#pragma once
#include "unscented_kalman_filter.h"


extern State state;

struct UKF;

void init_state(struct UKF *ukfh);

void state_update(struct UKF* ukfh);