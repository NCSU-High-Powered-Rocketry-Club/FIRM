#include "messages.h"
#include "mock_handler.h"
#include "usbd_cdc_if.h"

static uint8_t temp_buffer[COMMAND_READ_CHUNK_SIZE_BYTES];

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
  uint32_t total_crc_length = sizeof(header) + sizeof(identifier) + sizeof(payload_length) + payload_length;
  
  // Stack buffer for CRC calculation: header (4) + length (4) + payload
  memcpy(&temp_buffer[0], &header, sizeof(header));
  memcpy(&temp_buffer[sizeof(header)], &identifier, sizeof(identifier));
  memcpy(&temp_buffer[sizeof(header) + sizeof(identifier)], &payload_length, sizeof(payload_length));
  memcpy(&temp_buffer[sizeof(header) + sizeof(identifier) + sizeof(payload_length)], payload_and_crc, payload_length);
  
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

MessageIdentifier validate_message_identifier(uint16_t header, uint16_t identifier) {
  switch (header) {
    case MSGID_COMMAND_PACKET:
      switch (identifier) {
        // for cancel and mock, these have to go through the system manager
        // to determine if FIRM is in a valid state and what to do when switching
        // states
        case CMDID_CANCEL_REQUEST:
        case CMDID_MOCK_REQUEST:
          return MSGID_SYSTEM_MANAGER_REDIRECT;
        // other commands can be executed as normal.
        case CMDID_GET_DEVICE_CONFIG:
        case CMDID_GET_DEVICE_INFO:
        case CMDID_SET_DEVICE_CONFIG:
        case CMDID_REBOOT:
          return MSGID_COMMAND_PACKET;
        default:
          return MSGID_INVALID;
      }
    case MSGID_MOCK_PACKET:
      // mock packets must have the right identifier
      switch (identifier) {
        case MOCKID_BMP581:
        case MOCKID_ICM45686:
        case MOCKID_MMC5983MA:
        case MOCKID_SETTINGS:
          return MSGID_MOCK_PACKET;
        default:
          return MSGID_INVALID;
      }
    // should not have these headers at this point
    case MSGID_DATA_PACKET:
    case MSGID_SYSTEM_MANAGER_REDIRECT:
    case MSGID_RESPONSE_PACKET:
    case MSGID_INVALID:
    default:
      return MSGID_INVALID;
  }
}