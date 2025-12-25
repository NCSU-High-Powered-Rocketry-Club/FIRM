#include "commands.h"
#include "calibration.h"
#include "settings.h"
#include "utils.h"
#include "usb_serializer.h"
#include "stm32f4xx_hal_cortex.h"
#include <string.h>

// TODO: rename these with commands_ prefix

static DeviceProtocol_t select_protocol_from_settings(void) {
    // FIRM always outputs over USB. The protocol byte represents the "extra" protocol
    // to use when enabled; therefore, prefer any enabled non-USB protocol.
    // If multiple are enabled, pick a deterministic priority order.
    if (firmSettings.uart_transfer_enabled) {
        return UART;
    }
    if (firmSettings.i2c_transfer_enabled) {
        return I2C;
    }
    if (firmSettings.spi_transfer_enabled) {
        return SPI;
    }
    return USB;
}

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
        case CMD_CANCEL_ID: {
            // [CANCEL_MARKER][ACK (1 byte)]
            payload_buffer[0] = CMD_CANCEL_ID;
            *payload_len = 1;

            bool ack = (data != NULL) ? *(const bool*)data : true;
            payload_buffer[1] = ack ? 1 : 0;
            *payload_len += 1;
            break;
        }
        case CMD_GET_DEVICE_INFO: {
            // [DEVICE_INFO_MARKER][ID (8 bytes)][FIRMWARE_VERSION (8 bytes)][PADDING (16 bytes)]
            payload_buffer[0] = CMD_GET_DEVICE_INFO;
            *payload_len = 1;

            DeviceInfo_t empty_info;
            memset(&empty_info, 0, sizeof(empty_info));
            const DeviceInfo_t* info = (data != NULL) ? (const DeviceInfo_t*)data : &empty_info;

            memcpy(&payload_buffer[1], &info->id, sizeof(uint64_t));
            *payload_len += sizeof(uint64_t);

            // Ensure fixed length for strings by padding with 0
            memset(&payload_buffer[1 + sizeof(uint64_t)], 0, FIRMWARE_VERSION_LENGTH);
            strncpy((char*)&payload_buffer[1 + sizeof(uint64_t)], info->firmware_version, FIRMWARE_VERSION_LENGTH);
            *payload_len += FIRMWARE_VERSION_LENGTH;

            // Host expects 16 bytes of padding here (no port string).
            memset(&payload_buffer[1 + sizeof(uint64_t) + FIRMWARE_VERSION_LENGTH], 0, PORT_LENGTH);
            *payload_len += PORT_LENGTH;
            break;
        }
        case CMD_GET_DEVICE_CONFIG: {
            // [DEVICE_CONFIG_MARKER][NAME (32 bytes)][FREQUENCY (2 bytes)][PROTOCOL (1 byte)]
            payload_buffer[0] = CMD_GET_DEVICE_CONFIG;
            *payload_len = 1;

            DeviceConfig_t empty_config;
            memset(&empty_config, 0, sizeof(empty_config));
            const DeviceConfig_t* config = (data != NULL) ? (const DeviceConfig_t*)data : &empty_config;

            memset(&payload_buffer[1], 0, DEVICE_NAME_LENGTH);
            strncpy((char*)&payload_buffer[1], config->name, DEVICE_NAME_LENGTH);
            *payload_len += DEVICE_NAME_LENGTH;

            memcpy(&payload_buffer[1 + DEVICE_NAME_LENGTH], &config->frequency, sizeof(uint16_t));
            *payload_len += sizeof(uint16_t);

            payload_buffer[1 + DEVICE_NAME_LENGTH + sizeof(uint16_t)] = (uint8_t)config->protocol;
            *payload_len += 1;
            break;
        }
        case CMD_SET_DEVICE_CONFIG:
        case CMD_RUN_IMU_CALIBRATION:
        case CMD_RUN_MAG_CALIBRATION: {
            // [MARKER][SUCCESS (1 byte)]
            payload_buffer[0] = cmd_id;
            *payload_len = 1;
            
            bool success = *(bool*)data;
            payload_buffer[1] = success ? 1 : 0;
            *payload_len += 1;
            break;
        }
        case CMD_REBOOT: {
            // We don't send a reboot response, we just reboot.
            break;
        }
        default:
            break;
    }
}

void handle_command(const Command_t* cmd, const CommandContext_t* ctx, uint8_t* payload_buffer, uint8_t* payload_len) {
    if (cmd == NULL || payload_buffer == NULL || payload_len == NULL) {
        return;
    }

    // The command handler task remains responsible for serializing the payload.
    switch (cmd->id) {
        case CMD_GET_DEVICE_INFO: {
            DeviceInfo_t info;
            memset(&info, 0, sizeof(info));

            // Host expects: [MARKER][ID (8 LE bytes)][FIRMWARE_VERSION (8 bytes)][PADDING (16 bytes)]
            info.id = firmSettings.device_uid;

            // Fixed-length fields: create_response_payload() will pad with 0s and copy fixed widths.
            strncpy(info.firmware_version, firmSettings.firmware_version, FIRMWARE_VERSION_LENGTH);
            info.firmware_version[FIRMWARE_VERSION_LENGTH] = '\0';

            create_response_payload(CMD_GET_DEVICE_INFO, &info, payload_buffer, payload_len);
            break;
        }
        case CMD_GET_DEVICE_CONFIG: {
            DeviceConfig_t config;
            memset(&config, 0, sizeof(config));

            // Name is stored in settings with a 32-char limit + null terminator.
            // Ensure we always return a null-terminated local string.
            strncpy(config.name, firmSettings.device_name, DEVICE_NAME_LENGTH);
            config.name[DEVICE_NAME_LENGTH] = '\0';

            config.frequency = clamp_u16(
                firmSettings.frequency_hz,
                (uint16_t)FIRM_SETTINGS_FREQUENCY_MIN_HZ,
                (uint16_t)FIRM_SETTINGS_FREQUENCY_MAX_HZ
            );
            config.protocol = select_protocol_from_settings();

            create_response_payload(CMD_GET_DEVICE_CONFIG, &config, payload_buffer, payload_len);
            break;
        }
        case CMD_SET_DEVICE_CONFIG: {
            const DeviceConfig_t* new_config = &cmd->payload.set_config;
            // Update settings structure
            FIRMSettings_t updated_settings = firmSettings;
            strcpy(updated_settings.device_name, new_config->name);
            updated_settings.frequency_hz = clamp_u16(
                new_config->frequency,
                (uint16_t)FIRM_SETTINGS_FREQUENCY_MIN_HZ,
                (uint16_t)FIRM_SETTINGS_FREQUENCY_MAX_HZ
            );
            // Enable/disable protocols based on selected protocol
            updated_settings.usb_transfer_enabled = true; // always enabled
            updated_settings.uart_transfer_enabled = (new_config->protocol == UART);
            updated_settings.i2c_transfer_enabled = (new_config->protocol == I2C);
            updated_settings.spi_transfer_enabled = (new_config->protocol == SPI);
            bool success = settings_write_firm_settings(&updated_settings);
            create_response_payload(CMD_SET_DEVICE_CONFIG, &success, payload_buffer, payload_len);
            break;
        }
        case CMD_RUN_IMU_CALIBRATION: {
            bool success = calibration_run_imu(ctx);
            create_response_payload(CMD_RUN_IMU_CALIBRATION, &success, payload_buffer, payload_len);
            break;
        }
        case CMD_RUN_MAG_CALIBRATION: {
            bool success = calibration_run_mag(ctx);
            create_response_payload(CMD_RUN_MAG_CALIBRATION, &success, payload_buffer, payload_len);
            break;
        }
        case CMD_REBOOT: {
            // Reboot immediately. No response is generated.
            *payload_len = 0;
            HAL_NVIC_SystemReset();
            for (;;) {
            }
        }
        case CMD_CANCEL_ID: {
            // ACK the cancel command itself. The cancellation signal is handled by the task layer.
            bool ack = true;
            create_response_payload(CMD_CANCEL_ID, &ack, payload_buffer, payload_len);
            break;
        }
        default: {
            // Unknown/unimplemented command.
            // TODO: define an error payload and emit it here.
            *payload_len = 0;
            break;
        }
    }
}
