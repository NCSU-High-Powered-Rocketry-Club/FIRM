#include "commands.h"
#include "settings.h"
#include "utils.h"
#include "usb_serializer.h"
#include <string.h>

#ifdef TEST
  #include "stm32_hal_stubs.h" // for HAL_NVIC_SystemReset
#else
  #include "stm32f4xx_hal_cortex.h"
#endif

static DeviceProtocol select_protocol_from_settings(void) {
    // FIRM always outputs over USB. The protocol byte represents the "extra" protocol
    // to use when enabled. If the protocol is set to USB, that means only USB is used.
    if (firmSettings.uart_transfer_enabled) {
        return PROTOCOL_UART;
    }
    if (firmSettings.i2c_transfer_enabled) {
        return PROTOCOL_I2C;
    }
    if (firmSettings.spi_transfer_enabled) {
        return PROTOCOL_SPI;
    }
    return PROTOCOL_USB;
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
    // Check length and start markers
    if (len != CMD_PACKET_SIZE || buffer[0] != CMD_START_MARKER_1 || buffer[1] != CMD_START_MARKER_2) {
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
            if (protocol_byte >= PROTOCOL_USB && protocol_byte <= PROTOCOL_SPI) {
                command->payload.set_config.protocol = (DeviceProtocol)protocol_byte;
            } else {
                // Defaults to USB if invalid
                command->payload.set_config.protocol = PROTOCOL_USB;
            }
            break;
            
        case CMD_GET_DEVICE_INFO:
        case CMD_GET_DEVICE_CONFIG:
        case CMD_REBOOT:
        case CMD_MOCK:
        case CMD_CANCEL_ID:
            // These commands have no payload to parse
            break;
            
        default:
            return false;
    }

    return true;
}

void commands_update_parser(CommandsStreamParser_t* parser, const uint8_t* data, size_t data_len) {
  // Adds bytes to the parser buffer, dropping old bytes if necessary
  if (parser->len + data_len > sizeof(parser->buf)) {
    size_t drop_size = (parser->len + data_len) - sizeof(parser->buf);
    size_t remaining = parser->len - drop_size;
    if (remaining > 0) {
      memmove(parser->buf, parser->buf + drop_size, remaining);
    }
    parser->len = remaining;
  }

  memcpy(parser->buf + parser->len, data, data_len);
  parser->len += data_len;
}

/**
 * Extracts the next complete command packet from the parser buffer.
 * Removes consumed bytes from the parser buffer.
 */
bool commands_extract_next_packet(CommandsStreamParser_t* parser, uint8_t* command_buffer) {
  if (parser == NULL || command_buffer == NULL) {
      return false;
  }
  
  size_t current_byte = 0;
  
  while (current_byte + 1 < parser->len) {
    if (parser->buf[current_byte] != CMD_START_MARKER_1 || parser->buf[current_byte + 1] != CMD_START_MARKER_2) {
      current_byte++;
      continue;
    }

    if (current_byte + CMD_PACKET_SIZE > parser->len) {
      break; // Need more bytes
    }

    // Found potential packet, copy it
    memcpy(command_buffer, &parser->buf[current_byte], (size_t)CMD_PACKET_SIZE);
    
    // Remove consumed bytes from buffer  
    size_t remaining_len = parser->len - (current_byte + CMD_PACKET_SIZE);
    if (remaining_len > 0) {
        memmove(parser->buf, parser->buf + current_byte + CMD_PACKET_SIZE, remaining_len);
    }
    parser->len = remaining_len;
    
    return true;
  }

  // No complete packet found, trim leading garbage
  if (current_byte > 0 && current_byte < parser->len) {
    size_t remaining_len = parser->len - current_byte;
    memmove(parser->buf, parser->buf + current_byte, remaining_len);
    parser->len = remaining_len;
  }

  return false;
}

/**
 * Handles command execution and fills the response packet.
 */
static void commands_handle_command_impl(const Command_t* command, ResponsePacket* response_packet) {
    if (command == NULL || response_packet == NULL) {
        return;
    }

    switch (command->id) {
        case CMD_GET_DEVICE_INFO: {
            response_packet->response_id = MSGID_GET_DEVICE_INFO;
            DeviceInfo_t* info = &response_packet->response.device_info;
            memset(info, 0, sizeof(*info));

            info->id = firmSettings.device_uid;
            strncpy(info->firmware_version, firmSettings.firmware_version, FIRMWARE_VERSION_LENGTH);
            info->firmware_version[FIRMWARE_VERSION_LENGTH] = '\0';
            break;
        }
        case CMD_GET_DEVICE_CONFIG: {
            response_packet->response_id = MSGID_GET_DEVICE_CONFIG;
            DeviceConfig_t* config = &response_packet->response.device_config;
            memset(config, 0, sizeof(*config));

            strncpy(config->name, firmSettings.device_name, DEVICE_NAME_LENGTH);
            config->name[DEVICE_NAME_LENGTH] = '\0';

            config->frequency = clamp_u16(
                firmSettings.frequency_hz,
                (uint16_t)FIRM_SETTINGS_FREQUENCY_MIN_HZ,
                (uint16_t)FIRM_SETTINGS_FREQUENCY_MAX_HZ
            );
            config->protocol = select_protocol_from_settings();
            break;
        }
        case CMD_SET_DEVICE_CONFIG: {
            response_packet->response_id = MSGID_SET_DEVICE_CONFIG;
            const DeviceConfig_t* new_config = &command->payload.set_config;
            FIRMSettings_t updated_settings = firmSettings;
            strcpy(updated_settings.device_name, new_config->name);
            updated_settings.frequency_hz = clamp_u16(
                new_config->frequency,
                (uint16_t)FIRM_SETTINGS_FREQUENCY_MIN_HZ,
                (uint16_t)FIRM_SETTINGS_FREQUENCY_MAX_HZ
            );
            updated_settings.usb_transfer_enabled = true;
            updated_settings.uart_transfer_enabled = (new_config->protocol == PROTOCOL_UART);
            updated_settings.i2c_transfer_enabled = (new_config->protocol == PROTOCOL_I2C);
            updated_settings.spi_transfer_enabled = (new_config->protocol == PROTOCOL_SPI);
            settings_write_firm_settings(&updated_settings);
            break;
        }
        case CMD_MOCK: {
            response_packet->response_id = MSGID_MOCK;
            break;
        }
        case CMD_REBOOT: {
            response_packet->response_id = MSGID_REBOOT;
            HAL_NVIC_SystemReset();
            for (;;) {}
        }
        case CMD_CANCEL_ID: {
            response_packet->response_id = MSGID_CANCEL;
            break;
        }
        default: {
            break;
        }
    }
}

bool commands_process_and_respond(const uint8_t* command_buffer, ResponsePacket* response_packet) {
    if (command_buffer == NULL || response_packet == NULL) {
        return false;
    }

    // Parse the command
    Command_t command;
    if (!commands_parse_command(command_buffer, CMD_PACKET_SIZE, &command)) {
        return false;
    }

    // Initialize response packet
    memset(response_packet, 0, sizeof(*response_packet));

    // Execute command and fill response packet
    commands_handle_command_impl(&command, response_packet);

    return true;
}
