#pragma once

#include <stdbool.h>

#include "commands.h"

bool calibration_run_imu(const CommandContext_t* ctx);
bool calibration_run_mag(const CommandContext_t* ctx);
