#include "mock_handler.h"

MockPacketID process_mock_packet(uint16_t identifier, uint32_t length, uint8_t *received_bytes, uint8_t *mock_packet) {
  // if its the settings, return immediately because it has to be processed differently
  if (identifier == MOCKID_SETTINGS)
    return identifier;
  memcpy(mock_packet, received_bytes, length);
  return identifier;
}