#include "mock_handler.h"

MockPacketID process_mock_packet(const uint8_t *header_bytes, uint32_t length, uint8_t *received_bytes, uint8_t *mock_packet) {
  MockPacketID identifier = (uint16_t)(header_bytes[2]);

  // if its the settings, return immediately because it has to be processed differently
  if (identifier == MOCKID_SETTINGS)
    return identifier;
  memcpy(mock_packet, received_bytes, length);
  return identifier;
}