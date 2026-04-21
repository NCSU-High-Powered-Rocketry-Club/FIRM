#pragma once
#include "shared_data/system_settings.h"
#include "shared_data/identifiers.h"
#include "messages.h"
#include "settings_manager.h"
#include "transmit_frame.h"
#include <stdbool.h>
#include <string.h>

typedef void (*CommandSystemResetFn)(void *ctx);

void commands_register_system_reset(CommandSystemResetFn fn, void *ctx);

void commands_set_response_queue(void (*queue_send_fn)(TransmitFrame_t *transmit_frame));

void commands_set_sys_manager_send_cmd_fn(bool (*sys_manager_send_fn)(Identifiers_t id));

void dispatch_command(const uint8_t *command_bytes);