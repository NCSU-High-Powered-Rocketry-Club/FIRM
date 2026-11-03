#pragma once
#include "settings_manager.h"
#include "shared_data/identifiers.h"
#include "shared_data/system_settings.h"
#include "transmit_frame.h"
#include <stdbool.h>
#include <string.h>

typedef void (*CommandSystemResetFn)(void *ctx);

void commands_register_system_reset(CommandSystemResetFn fn, void *ctx);

void commands_set_response_queue(void (*queue_send_fn)(TransmitFrame_t *transmit_frame));

/**
 * @brief Payload bytes that follow the identifier, or -1 if the ID is not a command.
 */
int command_payload_len(uint8_t identifier_byte);

void dispatch_command(const uint8_t *command_bytes);
