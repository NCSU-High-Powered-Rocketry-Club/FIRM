#pragma once
#include "settings_manager.h"
#include "shared_data/identifiers.h"
#include "shared_data/system_settings.h"
#include "transmit_frame.h"
#include <stdbool.h>
#include <string.h>

typedef void (*CommandSystemResetFn)(void *ctx);
typedef void (*CommandCalibrationChangedFn)(void *ctx);

void commands_register_system_reset(CommandSystemResetFn fn, void *ctx);

/**
 * @brief Registers a callback run after a calibration command is written to settings, so the
 * new values can be loaded into the sensor drivers without a reboot.
 */
void commands_register_calibration_changed(CommandCalibrationChangedFn fn, void *ctx);

void commands_set_response_queue(void (*queue_send_fn)(TransmitFrame_t *transmit_frame));

void dispatch_command(const uint8_t *command_bytes);