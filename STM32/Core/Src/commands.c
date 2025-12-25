#include "commands.h"
#include "calibration.h"
#include "settings.h"
#include "utils.h"
#include "usb_serializer.h"
#include "stm32f4xx_hal_cortex.h"
#include <string.h>

static DeviceProtocol_t select_protocol_from_settings(void) {
    // FIRM always outputs over USB. The protocol byte represents the "extra" protocol
    // to use when enabled. If the protocol is set to USB, that means only USB is used.
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

/**
 * Parses a raw buffer into a Command_t structure.
 * 
 * @param buffer Pointer to the raw data buffer (must be at least 64 bytes).
 * @param len Length of the buffer (should be 64 for fixed packet size).
 * @param command Pointer to the Command_t structure to populate.
 * @return true if parsing was successful, false otherwise.
 */
static bool commands_parse_command(const uint8_t* buffer, uint32_t len, Command_t* command) {
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

    command->id = buffer[2];

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
            // These commands have no payload to parse
            break;
            
        default:
            return false;
    }

    return true;
}

/**
 * Appends data to the parser's buffer, dropping oldest data if overflow occurs.
 * 
 * @param parser Pointer to the CommandsStreamParser_t instance.
 * @param data Pointer to the incoming data chunk.
 * @param data_len Length of the incoming data chunk.
 */
static void commands_stream_parser_append_with_drop(CommandsStreamParser_t* parser, const uint8_t* data, size_t data_len) {
    if (data_len == 0) {
        return;
    }

    // If the incoming chunk is larger than our whole buffer, keep only the last bytes.
    if (data_len >= sizeof(parser->buf)) {
        memcpy(parser->buf, data + (data_len - sizeof(parser->buf)), sizeof(parser->buf));
        parser->len = sizeof(parser->buf);
        return;
    }

    // Drop oldest bytes on overflow.
    if (parser->len + data_len > sizeof(parser->buf)) {
        size_t drop = (parser->len + data_len) - sizeof(parser->buf);
        if (drop >= parser->len) {
            parser->len = 0;
        } else {
            memmove(parser->buf, parser->buf + drop, parser->len - drop);
            parser->len -= drop;
        }
    }

    memcpy(parser->buf + parser->len, data, data_len);
    parser->len += data_len;
}

void commands_stream_parser_init(CommandsStreamParser_t* parser) {
    if (parser == NULL) {
        return;
    }
    parser->len = 0;
}

size_t commands_stream_parser_feed(CommandsStreamParser_t* parser,
                                  const uint8_t* data,
                                  size_t data_len,
                                  CommandsStreamParserOnCommand on_command) {
    if (parser == NULL || data == NULL || data_len == 0 || on_command == NULL) {
        return 0;
    }

    commands_stream_parser_append_with_drop(parser, data, data_len);

    size_t emitted = 0;
    size_t pos = 0;

    while (pos + 1 < parser->len) {
        if (parser->buf[pos] != CMD_START_MARKER_1 || parser->buf[pos + 1] != CMD_START_MARKER_2) {
            pos++;
            continue;
        }

        if (pos + CMD_PACKET_SIZE > parser->len) {
            break; // Now we need more bytes
        }

        Command_t command;
        if (commands_parse_command(&parser->buf[pos], CMD_PACKET_SIZE, &command)) {
            on_command(&command);
            emitted++;
            pos += CMD_PACKET_SIZE;
        } else {
            // Bad packet, drop one byte and keep scanning.
            pos++;
        }
    }

    // Keeps only the tail
    if (pos > 0) {
        memmove(parser->buf, parser->buf + pos, parser->len - pos);
        parser->len -= pos;
    }

    return emitted;
}

void commands_create_response_payload(uint8_t command_id, const void* data, uint8_t* payload_buffer, uint8_t* payload_len) {
    if (payload_buffer == NULL || payload_len == NULL) {
        return;
    }

    payload_buffer[0] = command_id;
    *payload_len = 1;

    switch (command_id) {
        case CMD_CANCEL_ID: {
            // [CANCEL_MARKER][ACK (1 byte)]
            bool acknowledgement = (data != NULL) ? *(const bool*)data : true;
            payload_buffer[1] = acknowledgement ? 1 : 0;
            *payload_len += 1;
            break;
        }
        case CMD_GET_DEVICE_INFO: {
            // [DEVICE_INFO_MARKER][ID (8 bytes)][FIRMWARE_VERSION (8 bytes)][PADDING (16 bytes)]
            // We pass in a DeviceInfo pointer as data when handling this command
            const DeviceInfo_t* info = (const DeviceInfo_t*)data;

            memcpy(&payload_buffer[1], &info->id, sizeof(uint64_t));
            *payload_len += sizeof(uint64_t);

            // Ensure fixed length for strings by padding with 0
            memset(&payload_buffer[1 + sizeof(uint64_t)], 0, FIRMWARE_VERSION_LENGTH);
            strncpy((char*)&payload_buffer[1 + sizeof(uint64_t)], info->firmware_version, FIRMWARE_VERSION_LENGTH);
            *payload_len += FIRMWARE_VERSION_LENGTH;

            break;
        }
        case CMD_GET_DEVICE_CONFIG: {
            // [DEVICE_CONFIG_MARKER][NAME (32 bytes)][FREQUENCY (2 bytes)][PROTOCOL (1 byte)]
            const DeviceConfig_t* device_config = (const DeviceConfig_t*)data;

            memset(&payload_buffer[1], 0, DEVICE_NAME_LENGTH);
            strncpy((char*)&payload_buffer[1], device_config->name, DEVICE_NAME_LENGTH);
            *payload_len += DEVICE_NAME_LENGTH;

            memcpy(&payload_buffer[1 + DEVICE_NAME_LENGTH], &device_config->frequency, sizeof(uint16_t));
            *payload_len += sizeof(uint16_t);

            payload_buffer[1 + DEVICE_NAME_LENGTH + sizeof(uint16_t)] = (uint8_t)device_config->protocol;
            *payload_len += 1;
            break;
        }
        case CMD_SET_DEVICE_CONFIG:
        case CMD_RUN_IMU_CALIBRATION:
        case CMD_RUN_MAG_CALIBRATION: {
            // [MARKER][SUCCESS (1 byte)]
        
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

void commands_handle_command(const Command_t* command, const CommandContext_t* command_context, uint8_t* payload_buffer, uint8_t* payload_len) {
    if (command == NULL || payload_buffer == NULL || payload_len == NULL) {
        return;
    }

    switch (command->id) {
        case CMD_GET_DEVICE_INFO: {
            DeviceInfo_t info;
            memset(&info, 0, sizeof(info));

            // Host expects: [MARKER][ID (8 LE bytes)][FIRMWARE_VERSION (8 bytes)][PADDING (16 bytes)]
            info.id = firmSettings.device_uid;

            // Fixed-length fields: commands_create_response_payload() will pad with 0s and copy fixed widths.
            strncpy(info.firmware_version, firmSettings.firmware_version, FIRMWARE_VERSION_LENGTH);
            info.firmware_version[FIRMWARE_VERSION_LENGTH] = '\0';

            commands_create_response_payload(CMD_GET_DEVICE_INFO, &info, payload_buffer, payload_len);
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

            commands_create_response_payload(CMD_GET_DEVICE_CONFIG, &config, payload_buffer, payload_len);
            break;
        }
        case CMD_SET_DEVICE_CONFIG: {
            const DeviceConfig_t* new_config = &command->payload.set_config;
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
            commands_create_response_payload(CMD_SET_DEVICE_CONFIG, &success, payload_buffer, payload_len);
            break;
        }
        case CMD_RUN_IMU_CALIBRATION: {
            bool success = calibration_run_imu(command_context);
            commands_create_response_payload(CMD_RUN_IMU_CALIBRATION, &success, payload_buffer, payload_len);
            break;
        }
        case CMD_RUN_MAG_CALIBRATION: {
            bool success = calibration_run_mag(command_context);
            commands_create_response_payload(CMD_RUN_MAG_CALIBRATION, &success, payload_buffer, payload_len);
            break;
        }
        case CMD_REBOOT: {
            // Just reboot right away, don't worry about sending a response
            *payload_len = 0;
            HAL_NVIC_SystemReset();
            for (;;) {}
        }
        case CMD_CANCEL_ID: {
            // Acknowledge the cancel command itself. The cancellation signal is handled in firm_tasks
            bool acknowledgement = true;
            commands_create_response_payload(CMD_CANCEL_ID, &acknowledgement, payload_buffer, payload_len);
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
