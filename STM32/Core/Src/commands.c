#include "commands.h"
#include "usb_serializer.h"
#include <string.h>

bool parse_command(const uint8_t* buffer, uint32_t len, Command_t* command) {
    if (buffer == NULL || command == NULL || len != CMD_PACKET_SIZE) {
        return false;
    }

    // Check Start Markers
    if (buffer[0] != CMD_START_MARKER_1 || buffer[1] != CMD_START_MARKER_2) {
        return false;
    }

    // Calculate CRC over the packet (excluding the last 2 bytes which are the CRC)
    uint16_t received_crc = buffer[CMD_PACKET_SIZE - 2] | ((uint16_t)buffer[CMD_PACKET_SIZE - 1] << 8);
    uint16_t calculated_crc = crc16_ccitt(buffer, CMD_PACKET_SIZE - 2);

    if (calculated_crc != received_crc) {
        return false;
    }

    // Extract Command ID (3rd byte, index 2)
    command->id = buffer[2];

    // Parse Payload based on ID
    switch (command->id) {
        case CMD_SET_DEVICE_CONFIG:
            // Payload format: [NAME (32 bytes)][FREQUENCY (2 bytes)][PROTOCOL (1 byte)]
            memcpy(command->payload.set_config.name, &buffer[3], DEVICE_NAME_LENGTH);
            command->payload.set_config.name[DEVICE_NAME_LENGTH] = '\0'; // Ensure null termination

            // Frequency is little-endian
            command->payload.set_config.frequency = buffer[3 + DEVICE_NAME_LENGTH] | ((uint16_t)buffer[3 + DEVICE_NAME_LENGTH + 1] << 8);
            
            uint8_t protocol_byte = buffer[3 + DEVICE_NAME_LENGTH + 2];
            if (protocol_byte >= USB && protocol_byte <= SPI) {
                command->payload.set_config.protocol = (DeviceProtocol_t)protocol_byte;
            } else {
                // Defaults to USB if invalid
                command->payload.set_config.protocol = USB;
            }
            break;
            
        case CMD_GET_DEVICE_INFO:
        case CMD_GET_DEVICE_CONFIG:
        case CMD_RUN_IMU_CALIBRATION:
        case CMD_RUN_MAG_CALIBRATION:
        case CMD_REBOOT:
        case CMD_CANCEL_ID:
            // These commands have no payload to parse from the request
            break;
            
        default:
            return false; // Unknown command
    }

    return true;
}

void create_response_payload(uint8_t cmd_id, const void* data, uint8_t* payload_buffer, uint8_t* payload_len) {
    if (payload_buffer == NULL || payload_len == NULL) {
        return;
    }

    *payload_len = 0;

    switch (cmd_id) {
        case CMD_GET_DEVICE_INFO: {
            // [DEVICE_INFO_MARKER][ID (8 bytes)][FIRMWARE_VERSION (8 bytes)][PORT (16 bytes)]
            payload_buffer[0] = CMD_GET_DEVICE_INFO;
            *payload_len = 1;

            DeviceInfo_t default_info = {
                .id = 0x1234567890ABCDEF,
                .firmware_version = "v1.0.0",
                .port = "COM3"
            };
            const DeviceInfo_t* info = (data != NULL) ? (const DeviceInfo_t*)data : &default_info;

            memcpy(&payload_buffer[1], &info->id, sizeof(uint64_t));
            *payload_len += sizeof(uint64_t);

            // Ensure fixed length for strings by padding with 0
            memset(&payload_buffer[1 + sizeof(uint64_t)], 0, FIRMWARE_VERSION_LENGTH);
            strncpy((char*)&payload_buffer[1 + sizeof(uint64_t)], info->firmware_version, FIRMWARE_VERSION_LENGTH);
            *payload_len += FIRMWARE_VERSION_LENGTH;

            memset(&payload_buffer[1 + sizeof(uint64_t) + FIRMWARE_VERSION_LENGTH], 0, PORT_LENGTH);
            strncpy((char*)&payload_buffer[1 + sizeof(uint64_t) + FIRMWARE_VERSION_LENGTH], info->port, PORT_LENGTH);
            *payload_len += PORT_LENGTH;
            break;
        }
        case CMD_GET_DEVICE_CONFIG: {
            // [DEVICE_CONFIG_MARKER][NAME (32 bytes)][FREQUENCY (2 bytes)][PROTOCOL (1 byte)]
            payload_buffer[0] = CMD_GET_DEVICE_CONFIG;
            *payload_len = 1;

            DeviceConfig_t default_config = {
                .name = "FIRM Device",
                .frequency = 100,
                .protocol = USB
            };
            const DeviceConfig_t* config = (data != NULL) ? (const DeviceConfig_t*)data : &default_config;

            memset(&payload_buffer[1], 0, DEVICE_NAME_LENGTH);
            strncpy((char*)&payload_buffer[1], config->name, DEVICE_NAME_LENGTH);
            *payload_len += DEVICE_NAME_LENGTH;

            memcpy(&payload_buffer[1 + DEVICE_NAME_LENGTH], &config->frequency, sizeof(uint16_t));
            *payload_len += sizeof(uint16_t);

            payload_buffer[1 + DEVICE_NAME_LENGTH + sizeof(uint16_t)] = (uint8_t)config->protocol;
            *payload_len += 1;
            break;
        }
        case CMD_SET_DEVICE_CONFIG: {
            // [SET_DEVICE_CONFIG_MARKER][SUCCESS (1 byte)]
            payload_buffer[0] = CMD_SET_DEVICE_CONFIG;
            *payload_len = 1;
            
            bool success = (data != NULL) ? *(const bool*)data : true;
            payload_buffer[1] = success ? 1 : 0;
            *payload_len += 1;
            break;
        }
        case CMD_RUN_IMU_CALIBRATION:
        case CMD_RUN_MAG_CALIBRATION: {
            // [MARKER][CALIBRATION_COMPLETE (1 byte)][PROGRESS_PERCENTAGE (1 byte)]
            payload_buffer[0] = cmd_id;
            *payload_len = 1;

            CalibrationStatus_t default_status = {
                .calibration_complete = false,
                .progress_percentage = 0
            };
            const CalibrationStatus_t* status = (data != NULL) ? (const CalibrationStatus_t*)data : &default_status;

            payload_buffer[1] = status->calibration_complete ? 1 : 0;
            payload_buffer[2] = status->progress_percentage;
            *payload_len += 2;
            break;
        }
        default:
            break;
    }
}
