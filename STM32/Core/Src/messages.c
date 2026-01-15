#include "messages.h"

MessageIdentifier validate_message_header(uint16_t header) {
  // compare first 2 bytes of header
  switch (header) {
    case MSGID_COMMAND_PACKET:
    case MSGID_MOCK_PACKET:
      return header;
    // firm should never be receiving a message with a data packet or response packet header
    case MSGID_DATA_PACKET:
    case MSGID_RESPONSE_PACKET:
    default:
      return MSGID_INVALID;
  }
}

bool validate_message_crc16(uint16_t header, uint16_t identifier, uint32_t payload_length, const uint8_t* payload_and_crc) {
  // Extract CRC from the last 2 bytes of payload_and_crc
  uint16_t received_crc = payload_and_crc[payload_length] | ((uint16_t)payload_and_crc[payload_length + 1] << 8);
  
  // Calculate total message length for CRC verification
  uint32_t total_crc_length = sizeof(uint32_t) + sizeof(uint32_t) + payload_length;
  
  // Stack buffer for CRC calculation: header (4) + length (4) + payload
  uint8_t temp_buffer[total_crc_length];
  memcpy(&temp_buffer[0], &header, sizeof(uint16_t));
  memcpy(&temp_buffer[sizeof(uint16_t)], &identifier, sizeof(uint16_t));
  memcpy(&temp_buffer[sizeof(uint32_t)], &payload_length, sizeof(uint32_t));
  memcpy(&temp_buffer[sizeof(uint32_t) + sizeof(uint32_t)], payload_and_crc, payload_length);
  
  // Calculate CRC and compare
  uint16_t calculated_crc = crc16_ccitt(temp_buffer, total_crc_length);
  return (received_crc == calculated_crc);
}

void message_get_response_id(uint16_t header, uint16_t identifier, uint16_t* response_header_and_id) {
  // currently response packets will only be sent when FIRM is sent a command
  if (header == MSGID_COMMAND_PACKET) {
    // copies the last 2 bytes (command selection bytes) and sets first two to response packet id.
    response_header_and_id[0] = MSGID_RESPONSE_PACKET;
    response_header_and_id[1] = identifier;
  }
}

