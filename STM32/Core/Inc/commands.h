#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define COMMAND_READ_CHUNK_SIZE_BYTES 64
#define COMMAND_PAYLOAD_MAX_LEN_BYTES 56

// TODO clean these up
#define CMD_START_MARKER_1 0x55
#define CMD_START_MARKER_2 0xAA
#define CMD_PACKET_SIZE 64
#define CMD_CRC_SIZE 2

#define CMD_GET_DEVICE_INFO 0x01
#define CMD_GET_DEVICE_CONFIG 0x02
#define CMD_SET_DEVICE_CONFIG 0x03
#define CMD_REBOOT 0x04
#define CMD_MOCK 0x05
#define CMD_CANCEL_ID 0xFF

#define DEVICE_NAME_LENGTH 32
#define DEVICE_ID_LENGTH 8
#define FIRMWARE_VERSION_LENGTH 8
#define FREQUENCY_LENGTH 2

typedef enum {
  MSGID_GET_DEVICE_INFO = 0x01,
  MSGID_GET_DEVICE_CONFIG = 0x02,
  MSGID_SET_DEVICE_CONFIG = 0x03,
  MSGID_REBOOT = 0x04,
  MSGID_MOCK = 0x05,
  MSGID_CANCEL = 0xFF,
} MessageIdentifier;

typedef enum {
  PROTOCOL_USB = 0x01,
  PROTOCOL_UART = 0x02,
  PROTOCOL_I2C = 0x03,
  PROTOCOL_SPI = 0x04
} DeviceProtocol;

typedef struct {
  char name[DEVICE_NAME_LENGTH + 1];
  uint16_t frequency;
  DeviceProtocol protocol;
} DeviceConfig_t;

typedef struct {
  uint64_t id;
  char firmware_version[FIRMWARE_VERSION_LENGTH + 1];
} DeviceInfo_t;

typedef struct {
  MessageIdentifier response_id;
  union {
    DeviceConfig_t device_config;
    DeviceInfo_t device_info;
  } response;
} ResponsePacket;

/**
 * Structure representing a command received from the host.
 */
typedef struct {
  uint8_t id;
  union {
    DeviceConfig_t set_config;
  } payload;
} Command_t;

/**
 * Byte stream parser for incoming command data.
 */
typedef struct {
    uint8_t buf[128];
    size_t len;
} CommandsStreamParser_t;

/**
 * Feeds data into the stream parser.
 * 
 * @param parser Pointer to the CommandsStreamParser_t instance.
 * @param data Pointer to the incoming data chunk.
 * @param data_len Length of the incoming data chunk.
 */
void commands_update_parser(CommandsStreamParser_t* parser, const uint8_t* data, size_t data_len);

/**
 * Processes an incoming command and fills the response packet payload.
 * 
 * @param command_buffer Raw command buffer (must be CMD_PACKET_SIZE bytes).
 * @param response_packet Pointer to ResponsePacket structure to be filled with response data.
 * @return true if command was successfully parsed and processed, false otherwise.
 */
bool commands_process_and_respond(const uint8_t* command_buffer, ResponsePacket* response_packet);

/**
 * Extracts the next complete command packet from the parser buffer.
 * Removes consumed bytes from the parser buffer.
 * 
 * @param parser Pointer to the parser structure.
 * @param command_buffer Buffer to store the extracted command (must be CMD_PACKET_SIZE bytes).
 * @return true if a complete packet was extracted, false otherwise.
 */
bool commands_extract_next_packet(CommandsStreamParser_t* parser, uint8_t* command_buffer);

