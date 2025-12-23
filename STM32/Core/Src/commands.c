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
