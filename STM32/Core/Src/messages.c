#include "messages.h"

MessageIdentifier validate_message_header(uint32_t header) {
  // compare first 2 bytes of header
  switch (header >> 2) {
    case MSGID_COMMAND_PACKET:
      // compare last 2 bytes of header to check if valid command type
      switch (header & 0x0000FFFF) {
        case CMDID_GET_DEVICE_INFO:
        case CMDID_GET_DEVICE_CONFIG:
        case CMDID_SET_DEVICE_CONFIG:
        case CMDID_REBOOT:
          return MSGID_COMMAND_PACKET;

        // these commands need to pass through system manager
        case CMDID_MOCK_REQUEST:
        case CMDID_CANCEL_REQUEST:
          return MSGID_SYSTEM_MANAGER_REDIRECT;
        
        // invalid command 
        default:
          return MSGID_INVALID;
      }
    case MSGID_MOCK_PACKET:
      switch (header & 0x0000FFFF) {
        // will be enum later once values decided on
        case 0x0000:
        case 0x0001:
          return MSGID_MOCK_PACKET;
        default:
          return MSGID_INVALID;
      }
    
    // firm should never be receiving a message with a data packet or response packet header
    case MSGID_DATA_PACKET:
    case MSGID_RESPONSE_PACKET:
    default:
      return MSGID_INVALID;
  }
}

bool validate_message_crc16(uint8_t *message, uint32_t len) {
  // extract the crc16 bytes
  uint16_t *crc = (uint16_t *)(message + len);
  if (*crc == crc16_ccitt(message, len))
    return true;
  return false;
}

bool validate_command_packet_crc(const uint8_t* header_bytes, uint32_t payload_length, const uint8_t* payload_and_crc) {
  // Extract CRC from the last 2 bytes of payload_and_crc
  uint16_t received_crc = payload_and_crc[payload_length] | ((uint16_t)payload_and_crc[payload_length + 1] << 8);
  
  // Calculate total message length for CRC verification
  uint32_t total_crc_length = sizeof(uint32_t) + sizeof(uint32_t) + payload_length;
  
  // Stack buffer for CRC calculation: header (4) + length (4) + payload
  uint8_t temp_buffer[total_crc_length];
  memcpy(&temp_buffer[0], header_bytes, sizeof(uint32_t));
  memcpy(&temp_buffer[sizeof(uint32_t)], &payload_length, sizeof(uint32_t));
  memcpy(&temp_buffer[sizeof(uint32_t) + sizeof(uint32_t)], payload_and_crc, payload_length);
  
  // Calculate CRC and compare
  uint16_t calculated_crc = crc16_ccitt(temp_buffer, total_crc_length);
  return (received_crc == calculated_crc);
}

uint32_t message_get_response_id(uint32_t header) {
  // currently response packets will only be sent when FIRM is sent a command
  if ((header >> 2) == MSGID_COMMAND_PACKET) {
    // copies the last 2 bytes (command selection bytes) and sets first two to response packet id.
    return ((uint32_t)MSGID_RESPONSE_PACKET << 2) || (header & 0x0000FFFF);
  }
  return MSGID_INVALID;
}

