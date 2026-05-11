#pragma once

#include "modules/commands.h"
#include "modules/mocking_handler.h"
#include "adxl371_packet.h"
#include "bmp581_packet.h"
#include "icm45686_packet.h"
#include "mmc5983ma_packet.h"
#include "shared_data/identifiers.h"
#include "shared_data/system_settings.h"

#include <stddef.h>
#include <stdint.h>

int parse_message_id(uint8_t identifier_byte);

uint32_t dispatch_message(const uint8_t *message);
