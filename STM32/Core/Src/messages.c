#include "messages.h"

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
        case CMDID_SET_IMU_CALIBRATON:
        case CMDID_SET_MAG_CALIBRATON:
        case CMDID_REBOOT:
          return MSGID_COMMAND_PACKET;
        default:
          return MSGID_INVALID;
      }
    case MSGID_MOCK_PACKET:
      // mock packets must have the right identifier
      switch (identifier) {
        case (uint16_t)'B':
        case (uint16_t)'I':
        case (uint16_t)'M':
        case (uint16_t)'H':
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

bool usb_parse_message_meta(const uint8_t *meta_bytes, size_t meta_len, UsbMessageMeta *out) {
  if (meta_bytes == NULL || out == NULL) {
    return false;
  }
  if (meta_len < (sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint32_t))) {
    return false;
  }
  memcpy(&out->header, &meta_bytes[0], sizeof(out->header));
  memcpy(&out->identifier, &meta_bytes[sizeof(out->header)], sizeof(out->identifier));
  memcpy(&out->payload_length, &meta_bytes[sizeof(out->header) + sizeof(out->identifier)], sizeof(out->payload_length));
  return true;
}

UsbMessageType usb_interpret_usb_message(const UsbMessageMeta *meta) {
  MessageIdentifier header_type = validate_message_header(meta->header);
  if (header_type == MSGID_INVALID) {
    return USBMSG_INVALID;
  }

  MessageIdentifier id_type = validate_message_identifier(meta->header, meta->identifier);
  if (id_type == MSGID_INVALID) {
    return USBMSG_INVALID;
  }

  // Mock packet handling
  if (meta->header == MSGID_MOCK_PACKET) {
    // mock settings packet
    if (meta->identifier == (uint16_t)'H') {
      return USBMSG_MOCK_SETTINGS;
    }
    // Mock sensor packets
    return USBMSG_MOCK_SENSOR;
  }

  switch (id_type) {
    case MSGID_SYSTEM_MANAGER_REDIRECT:
      return USBMSG_SYSTEM_REQUEST;
    case MSGID_COMMAND_PACKET:
      return USBMSG_COMMAND;
    default:
      return USBMSG_INVALID;
  }
}