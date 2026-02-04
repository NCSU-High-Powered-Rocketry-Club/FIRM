#include "commands.h"
#include "messages.h"
#include "settings.h"
#include "utils.h"
#include <string.h>

static CommandSystemResetFn g_system_reset_fn = NULL;
static void *g_system_reset_ctx = NULL;

void commands_register_system_reset(CommandSystemResetFn fn, void *ctx) {
  g_system_reset_fn = fn;
  g_system_reset_ctx = ctx;
}



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

static bool set_device_config(DeviceConfig *new_config) {
  FIRMSettings_t updated_settings = firmSettings;
  strncpy(updated_settings.device_name, new_config->name, DEVICE_NAME_LENGTH);
  updated_settings.frequency_hz = clamp_u16(
      new_config->frequency,
      (uint16_t)FIRM_SETTINGS_FREQUENCY_MIN_HZ,
      (uint16_t)FIRM_SETTINGS_FREQUENCY_MAX_HZ
  );
  // Enable/disable protocols based on selected protocol
  updated_settings.usb_transfer_enabled = true; // always enabled
  updated_settings.uart_transfer_enabled = (new_config->protocol == PROTOCOL_UART);
  updated_settings.i2c_transfer_enabled = (new_config->protocol == PROTOCOL_I2C);
  updated_settings.spi_transfer_enabled = (new_config->protocol == PROTOCOL_SPI);
  return settings_write_firm_settings(&updated_settings);
}


uint32_t execute_command(CommandIdentifier identifier, uint8_t *data, uint32_t data_len, ResponsePacket* response_packet) {
  switch (identifier) {
    case CMDID_REBOOT: // system reboots, dont worry about sending response packet
      if (g_system_reset_fn != NULL) {
        g_system_reset_fn(g_system_reset_ctx);
      }
      return 0;

    case CMDID_SET_DEVICE_CONFIG: {
      // Payload format: [NAME (32 bytes)][FREQUENCY (2 bytes)][PROTOCOL (1 byte)]
      DeviceConfig device_conf;
      memcpy(device_conf.name, data, DEVICE_NAME_LENGTH);

      // Frequency is little-endian
      device_conf.frequency = data[1 + DEVICE_NAME_LENGTH] | ((uint16_t)data[2 + DEVICE_NAME_LENGTH] << 8);

      uint8_t protocol_byte = data[2 + DEVICE_NAME_LENGTH];
      if (protocol_byte >= PROTOCOL_USB && protocol_byte <= PROTOCOL_SPI) {
        device_conf.protocol = (DeviceProtocol)protocol_byte;
      } else {
        // Defaults to USB if invalid
        device_conf.protocol = PROTOCOL_USB;
      }
      // set settings, and set response packet to sucess or failure
      response_packet->success.b = set_device_config(&device_conf);
      return 1;
    }
    case CMDID_GET_DEVICE_INFO:
      // Response payload format: [ID (8 LE bytes)][FIRMWARE_VERSION (8 bytes)]
      response_packet->device_info.id = firmSettings.device_uid;
      strncpy(response_packet->device_info.firmware_version, firmSettings.firmware_version, FIRMWARE_VERSION_LENGTH);
      return sizeof(response_packet->device_info);
    case CMDID_GET_DEVICE_CONFIG:
      // Response payload format: [NAME (32 bytes)][FREQUENCY (2 bytes)][PROTOCOL (1 byte)]
      strncpy(response_packet->device_config.name, firmSettings.device_name, DEVICE_NAME_LENGTH);
      response_packet->device_config.frequency = clamp_u16(firmSettings.frequency_hz, (uint16_t)FIRM_SETTINGS_FREQUENCY_MIN_HZ, (uint16_t)FIRM_SETTINGS_FREQUENCY_MAX_HZ);
      response_packet->device_config.protocol = select_protocol_from_settings();
      return sizeof(response_packet->device_config);
    case CMDID_SET_IMU_CALIBRATON: {
      // Payload format: [AccelCalibration_t][GyroCalibration_t]
      AccelCalibration_t new_accel_calibration;
      GyroCalibration_t new_gyro_calibration;
      memcpy(&new_accel_calibration, data, sizeof(AccelCalibration_t));
      memcpy(&new_gyro_calibration, &data[sizeof(AccelCalibration_t)], sizeof(GyroCalibration_t));
      response_packet->success.b = settings_write_calibration_settings(&new_accel_calibration, &new_gyro_calibration, NULL);
      return 1;
    }
    case CMDID_SET_MAG_CALIBRATON: {
      // Payload format: CalibrationSettings struct
      MagCalibration_t new_mag_calibration;
      memcpy(&new_mag_calibration, data, sizeof(MagCalibration_t));
      response_packet->success.b = settings_write_calibration_settings(NULL, NULL, &new_mag_calibration);
      return 1;
    }
    case CMDID_MOCK_REQUEST:
    case CMDID_CANCEL_REQUEST:
    default:
      return 0;
  }
}

uint32_t commands_execute_to_response(uint16_t request_identifier,
                                     uint8_t *payload_bytes,
                                     uint32_t payload_len,
                                     uint16_t *out_response_header,
                                     uint16_t *out_response_identifier,
                                     ResponsePacket *out_response_payload) {
  *out_response_header = MSGID_RESPONSE_PACKET;
  *out_response_identifier = request_identifier;
  return execute_command((CommandIdentifier)request_identifier, payload_bytes, payload_len, out_response_payload);
}