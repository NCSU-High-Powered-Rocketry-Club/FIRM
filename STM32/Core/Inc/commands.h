#pragma once

#include <stdint.h>
#include <stdbool.h>

#define CMD_START_MARKER_1 0x55
#define CMD_START_MARKER_2 0xAA
#define CMD_PACKET_SIZE 64
#define CMD_CRC_SIZE 2

#define CMD_GET_DEVICE_INFO 0x01
#define CMD_GET_DEVICE_CONFIG 0x02
#define CMD_SET_DEVICE_CONFIG 0x03
#define CMD_RUN_IMU_CALIBRATION 0x04
#define CMD_RUN_MAG_CALIBRATION 0x05
#define CMD_REBOOT 0x06
#define CMD_CANCEL_ID 0xFF // Special ID for cancel command

#define DEVICE_NAME_LENGTH 32
#define DEVICE_ID_LENGTH 8
#define FIRMWARE_VERSION_LENGTH 8
#define PORT_LENGTH 16
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

typedef struct {
    uint64_t id;
    char firmware_version[FIRMWARE_VERSION_LENGTH + 1];
    char port[PORT_LENGTH + 1];
} DeviceInfo_t;

typedef struct {
    bool calibration_complete;
    uint8_t progress_percentage;
} CalibrationStatus_t;

typedef struct {
    uint8_t id;
    union {
        DeviceConfig_t set_config;
        // Add other command payloads here as needed
    } payload;
} Command_t;

/**
 * @brief Parses a raw buffer into a Command_t structure.
 * 
 * @param buffer Pointer to the raw data buffer (must be at least 64 bytes).
 * @param len Length of the buffer (should be 64 for fixed packet size).
 * @param command Pointer to the Command_t structure to populate.
 * @return true if parsing was successful, false otherwise.
 */
bool parse_command(const uint8_t* buffer, uint32_t len, Command_t* command);

/**
 * @brief Creates the payload for a response packet based on the command ID.
 * 
 * @param cmd_id The ID of the command to respond to.
 * @param data Pointer to the data structure required for the response (e.g., DeviceInfo_t*).
 * @param payload_buffer Buffer to write the payload to.
 * @param payload_len Pointer to store the length of the generated payload.
 */
void create_response_payload(uint8_t cmd_id, const void* data, uint8_t* payload_buffer, uint8_t* payload_len);

