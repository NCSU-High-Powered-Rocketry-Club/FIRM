#pragma once
#include "commands.h"
#include "mocking_handler.h"
#include "adxl371_packet.h"
#include "bmp581_packet.h"
#include "icm45686_packet.h"
#include "mmc5983ma_packet.h"
#include "shared_data/identifiers.h"
#include "shared_data/system_settings.h"

#include <stddef.h>

size_t parse_message_id(uint8_t identifier_byte);

uint32_t dispatch_message(const uint8_t *message);
