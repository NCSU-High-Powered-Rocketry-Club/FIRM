#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define COMMAND_READ_CHUNK_SIZE_BYTES 64

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
    USB = 0x01,
    UART = 0x02,
    I2C = 0x03,
    SPI = 0x04
} DeviceProtocol_t;

typedef struct {
    char name[DEVICE_NAME_LENGTH + 1]; // + 1 for null terminator
    uint16_t frequency;
    DeviceProtocol_t protocol;
} DeviceConfig_t;

/**
 * Structure representing device information.
 */
typedef struct {
    uint64_t id;
    char firmware_version[FIRMWARE_VERSION_LENGTH + 1];
} DeviceInfo_t;

/**
 * Structure representing a command received from the host.
 */
typedef struct {
    uint8_t id;
    union {
        DeviceConfig_t set_config;
        // We can add other command payloads if we need to in the future
    } payload;
} Command_t;

/**
 * Cancellation context for long-running commands.
 *
 * The command runner snapshots a monotonically increasing cancellation sequence.
 * A command is considered cancelled if the global sequence differs from the snapshot.
 */
typedef struct {
    volatile uint32_t* cancel_seq;
    uint32_t snapshot;
} CommandCancelCtx_t;

/**
 * Structure representing context for command execution, including cancellation support.
 */
typedef struct {
    bool (*is_cancelled)(const CommandCancelCtx_t* cancel_context);
    const CommandCancelCtx_t* cancel_context;
} CommandContext_t;

/**
 * Byte stream parser for incoming command data.
 */
typedef struct {
    uint8_t buf[128];
    size_t len;
} CommandsStreamParser_t;

typedef void (*CommandsStreamParserOnCommand)(const Command_t* cmd);

/**
 * Feeds data into the stream parser, emitting parsed commands via callback.
 * 
 * @param parser Pointer to the CommandsStreamParser_t instance.
 * @param data Pointer to the incoming data chunk.
 * @param data_len Length of the incoming data chunk.
 * @param on_command Callback function to be called for each parsed command.
 */
void commands_update_parser(CommandsStreamParser_t* parser,
                                  const uint8_t* data,
                                  size_t data_len,
                                  CommandsStreamParserOnCommand on_command);

/**
 * Creates the payload for a response packet based on the command ID.
 * 
 * @param command_id The ID of the command to respond to.
 * @param data Pointer to the data structure required for the response (e.g., DeviceInfo_t*).
 * @param payload_buffer Buffer to write the payload to.
 * @param payload_len Pointer to store the length of the generated payload.
 */
void commands_create_response_payload(uint8_t command_id, const void* data, uint8_t* payload_buffer, uint8_t* payload_len);

/**
 * Handles an incoming command, makes the response payload, and writes it to the provided buffer.
 * 
 * @param command Pointer to the Command_t structure representing the received command.
 * @param command_context Pointer to the CommandContext_t for cancellation support.
 * @param payload_buffer Buffer to write the response payload to.
 * @param payload_len Pointer to store the length of the generated payload.
 */
void commands_handle_command(const Command_t* command, const CommandContext_t* command_context, uint8_t* payload_buffer, uint8_t* payload_len);

