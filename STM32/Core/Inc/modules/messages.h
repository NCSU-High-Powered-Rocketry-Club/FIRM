#pragma once

#include "modules/commands.h"

#include <stdint.h>

int parse_message_id(uint8_t identifier_byte);

void dispatch_message(const uint8_t *message);
