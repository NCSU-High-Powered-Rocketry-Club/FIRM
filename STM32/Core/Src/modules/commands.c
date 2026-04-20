#include "commands.h"

static CommandSystemResetFn g_system_reset_fn = NULL;
static void *g_system_reset_ctx = NULL;

static uint16_t clamp_u16(uint16_t value, uint16_t min_value, uint16_t max_value) {
  if (value < min_value)
    return min_value;
  if (value > max_value)
    return max_value;
  return value;
}

void commands_register_system_reset(CommandSystemResetFn fn, void *ctx) {
  g_system_reset_fn = fn;
  g_system_reset_ctx = ctx;
}

void dispatch_command(const uint8_t *command_bytes) {
  const SystemSettings_t *settings = get_settings();
  Identifiers_t command_id = (Identifiers_t)command_bytes[0];
  command_bytes += 1; // move the pointer so that you can simply copy command_bytes to get the data

  switch (command_id) {
  case ID_GET_DEVICE_INFO:
  case ID_GET_DEVICE_CONFIG:
  case ID_SET_DEVICE_CONFIG: {
    // copy device info into DeviceConfig_t struct
    DeviceConfig_t new_conf;
    memcpy(&new_conf, command_bytes, sizeof(DeviceConfig_t));

    // get copy of current settings, to modify with new config
    SystemSettings_t updated_settings = *get_settings();
    // copy in name, frequency, and communication bools
    strncpy(updated_settings.device_name, new_conf.device_name, FIRM_DEVICE_NAME_LENGTH);
    updated_settings.frequency_hz =
        clamp_u16(new_conf.frequency_hz, (uint16_t)FIRM_SETTINGS_FREQUENCY_MIN_HZ,
                  (uint16_t)FIRM_SETTINGS_FREQUENCY_MAX_HZ);
    updated_settings.usb_transfer_enabled = true; // usb is always enabled
    updated_settings.uart_transfer_enabled = new_conf.uart_transfer_enabled;
    updated_settings.i2c_transfer_enabled = new_conf.i2c_transfer_enabled;
    updated_settings.spi_transfer_enabled = new_conf.spi_transfer_enabled;

    // update the settings module with the new, updated settings
    settings_write_firm_settings(&updated_settings);
    break;
  }

  case ID_REBOOT:
    if (g_system_reset_fn != NULL)
      g_system_reset_fn(g_system_reset_ctx);
    break;

  case ID_MOCK_REQUEST:

  case ID_SET_MAG_CALIBRATON: {
    Calibration_t new_mag_calibration;
    memcpy(&new_mag_calibration, command_bytes, sizeof(Calibration_t));
    settings_write_calibration(NULL, NULL, &new_mag_calibration, NULL);
    break;
  }

  case ID_SET_IMU_CALIBRATON: {
    Calibration_t new_accel_calibration;
    Calibration_t new_gyro_calibration;
    memcpy(&new_accel_calibration, command_bytes, sizeof(Calibration_t));
    memcpy(&new_gyro_calibration, command_bytes + sizeof(Calibration_t), sizeof(Calibration_t));
    settings_write_calibration(&new_accel_calibration, &new_gyro_calibration, NULL, NULL);
    break;
  }

  case ID_GET_CALIBRATION:
  case ID_CANCEL_REQUEST:

  case CMDID_GET_DEVICE_INFO: {
    // Response payload format: [ID (8 LE bytes)][FIRMWARE_VERSION (8 bytes)]
    response_packet->device_info.id = settings->device_uid;
    strncpy(response_packet->device_info.firmware_version, settings->firmware_version,
            FIRMWARE_VERSION_LENGTH);
    return sizeof(response_packet->device_info);
  }
  case CMDID_GET_DEVICE_CONFIG: {
    // Response payload format: [NAME (32 bytes)][FREQUENCY (2 bytes)][PROTOCOL (1 byte)]
    strncpy(response_packet->device_config.name, settings->device_name, DEVICE_NAME_LENGTH);
    response_packet->device_config.frequency =
        clamp_u16(settings->frequency_hz, (uint16_t)FIRM_SETTINGS_FREQUENCY_MIN_HZ,
                  (uint16_t)FIRM_SETTINGS_FREQUENCY_MAX_HZ);
    response_packet->device_config.protocol = select_protocol_from_settings();
    return sizeof(response_packet->device_config);
  }

  case CMDID_GET_CALIBRATION: {
    // Payload format: [Accel Calibration][Gyro Calibration][Mag Calibration][High-g Calibration]
    // (in row-major)
    CalibrationSettings_t all_calibration_data = {
        settings->accel_cal,
        settings->gyro_cal,
        settings->mag_cal,
        settings->high_g_cal,
    };
    response_packet->calibration_settings = all_calibration_data;
    return sizeof(response_packet->calibration_settings);
  }

  default:
    return;
  }
}