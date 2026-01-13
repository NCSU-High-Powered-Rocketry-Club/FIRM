#pragma once
#include "commands.h"
#include "utils.h"
#include <string.h>

/**
 * The first two bytes of the header field of a message packet being sent or received.
 */
typedef enum {
  MSGID_INVALID = 0xFFFF,
  MSGID_SYSTEM_MANAGER_REDIRECT = 0x0001,
  MSGID_DATA_PACKET = 0xA55A,
  MSGID_RESPONSE_PACKET = 0x5AA5,
  MSGID_COMMAND_PACKET = 0x6BB6,
  MSGID_MOCK_PACKET = 0xB66B,
} MessageIdentifier;

/**
 * Checks a received message's header for valid identifier bytes, and returns the type
 * of message
 * @param header the 4 header bytes as an unsigned integer
 * @return the message identifier type of the header
 */
MessageIdentifier validate_message_header(uint32_t header);


uint32_t message_get_response_id(uint32_t header);
/**
 * Validates CRC16 across separate header, length, and payload buffers without copying.
 * 
 * @param header_bytes Pointer to 4-byte header identifier
 * @param payload_length Length of the payload
 * @param payload_and_crc Pointer to payload + 2 CRC bytes
 * @return true if CRC is valid, false otherwise
 */
bool validate_message_crc(const uint8_t* header_bytes, uint32_t payload_length, const uint8_t* payload_and_crc);