#pragma once

#include <stdbool.h>

#include "commands.h"

/**
 * Runs the IMU calibration process.
 * 
 * @param command_context Pointer to the command context for cancellation support.
 * @return true if calibration was successful, false if cancelled or failed.
 */
bool calibration_run_imu(const CommandContext_t* command_context);

/**
 * Runs the magnetometer calibration process.
 * 
 * @param command_context Pointer to the command context for cancellation support.
 * @return true if calibration was successful, false if cancelled or failed.
 */
bool calibration_run_mag(const CommandContext_t* command_context);
