#pragma once
#include "shared_data/system_settings.h"
#include "messages.h"
#include "settings_manager.h"
#include <stdbool.h>
#include <string.h>

typedef void (*CommandSystemResetFn)(void *ctx);

void commands_register_system_reset(CommandSystemResetFn fn, void *ctx);

void dispatch_command(const uint8_t *command_bytes);