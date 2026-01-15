#pragma once
#include <stdint.h>
#include <string.h>

typedef enum {
  MOCKID_BMP581,
  MOCKID_ICM45686,
  MOCKID_MMC5983MA,
  MOCKID_SETTINGS,
} MockPacketID;

MockPacketID process_mock_packet(uint16_t identifier, uint32_t length, uint8_t *received_bytes, uint8_t *mock_packet);