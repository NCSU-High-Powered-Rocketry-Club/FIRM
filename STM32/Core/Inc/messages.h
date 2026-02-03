#pragma once
#include "commands.h"
#include "utils.h"
#include <string.h>
/**
 * The first two bytes of the header field of a message packet being sent or received.
 */
typedef enum {
  MSGID_INVALID = 0xFFFF,
  MSGID_SYSTEM_MANAGER_REDIRECT = (uint16_t)0x0001,
  MSGID_DATA_PACKET = 0xA55A,
  MSGID_RESPONSE_PACKET = 0x5AA5,
  MSGID_COMMAND_PACKET = 0xB66B,
  MSGID_MOCK_PACKET = 0x6BB6,
} MessageIdentifier;

typedef enum {
  USBMSG_INVALID = 0,
  USBMSG_COMMAND,
  USBMSG_SYSTEM_REQUEST,
  USBMSG_MOCK_SENSOR,
  USBMSG_MOCK_SETTINGS,
} UsbMessageType;

typedef struct {
  uint16_t header;
  uint16_t identifier;
  uint32_t payload_length;
} UsbMessageMeta;

/**
 * Checks a received message's header for valid identifier bytes, and returns the type
 * of message
 * @param header the 4 header bytes as an unsigned integer
 * @return the message identifier type of the header
 */
MessageIdentifier validate_message_header(uint16_t header);

MessageIdentifier validate_message_identifier(uint16_t header, uint16_t identifier);

/**
 * Validates CRC16 across separate header, length, and payload buffers without copying.
 * 
 * @param header_bytes Pointer to 4-byte header identifier
 * @param payload_length Length of the payload
 * @param payload_and_crc Pointer to payload + 2 CRC bytes
 * @return true if CRC is valid, false otherwise
 */
bool validate_message_crc16(uint16_t header, uint16_t identifier, uint32_t payload_length, const uint8_t* payload_and_crc);

/**
 * Parses the 8-byte USB meta header:
 *   [header (2)][identifier (2)][payload_length (4)]
 */
bool usb_parse_message_meta(const uint8_t *meta_bytes, size_t meta_len, UsbMessageMeta *out);

/**
 * Interprets a single USB message (meta + payload + CRC bytes). This is pure logic.
 *
 * Notes:
 * - For command packets and mock settings packets, CRC16 is validated.
 * - For mock sensor packets (B/I/M), CRC is ignored to match current device behavior.
 */
UsbMessageType usb_interpret_usb_message(const UsbMessageMeta *meta);