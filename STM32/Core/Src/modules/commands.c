#include "commands.h"

static CommandSystemResetFn g_system_reset_fn = NULL;
static void *g_system_reset_ctx = NULL;
static void (*queue_send)(TransmitFrame_t *transmit_frame) = NULL;
static bool (*sys_manager_send_cmd)(Identifiers_t id) = NULL;

static uint16_t clamp_u16(uint16_t value, uint16_t min_value, uint16_t max_value) {
  if (value < min_value)
    return min_value;
  if (value > max_value)
    return max_value;
  return value;
}

static void send_response(Identifiers_t id, uint8_t *response_data, uint16_t response_len) {
  TransmitFrame_t transmit_frame;
  
  // response length does NOT include identifier byte
  transmit_frame.payload_len = response_len;

  // first byte is identifier byte, after is the response data
  transmit_frame.payload[0] = (uint8_t)id;
  memcpy(&transmit_frame.payload[1], response_data, response_len);
  queue_send(&transmit_frame);
}

void commands_register_system_reset(CommandSystemResetFn fn, void *ctx) {
  g_system_reset_fn = fn;
  g_system_reset_ctx = ctx;
}

void commands_set_response_queue(void (*queue_send_fn)(TransmitFrame_t *transmit_frame)) {
  queue_send = queue_send_fn;
}

void commands_set_sys_manager_send_cmd_fn(bool (*sys_manager_send_fn)(Identifiers_t id)) {
  sys_manager_send_cmd = sys_manager_send_fn;
}

void dispatch_command(const uint8_t *command_bytes) {
  const SystemSettings_t *settings = get_settings();
  Identifiers_t command_id = (Identifiers_t)command_bytes[0];
  command_bytes += 1; // move the pointer so that you can simply copy command_bytes to get the data

  switch (command_id) {
  case ID_GET_DEVICE_INFO: {
    DeviceInfo_t device_info;
    device_info.device_uid = settings->device_uid;
    memcpy(&device_info.firmware_version, &settings->firmware_version, FIRM_FIRMWARE_VERSION_LEN);
    send_response(ID_GET_DEVICE_INFO, (uint8_t *)&device_info, sizeof(DeviceInfo_t));
    break;
  }
    
  case ID_GET_DEVICE_CONFIG: {
    DeviceConfig_t device_config;
    device_config.frequency_hz = settings->frequency_hz;
    memcpy(device_config.device_name, settings->device_name, FIRM_DEVICE_NAME_LENGTH);    
    device_config.usb_transfer_enabled = settings->usb_transfer_enabled;
    device_config.uart_transfer_enabled = settings->uart_transfer_enabled;
    device_config.i2c_transfer_enabled = settings->i2c_transfer_enabled;
    device_config.spi_transfer_enabled = settings->spi_transfer_enabled;
    send_response(ID_GET_DEVICE_CONFIG, (uint8_t *)&device_config, sizeof(DeviceConfig_t));
    break;
  }

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
    if (sys_manager_send_cmd != NULL) {
      if (sys_manager_send_cmd(ID_MOCK_REQUEST)) {
        send_response(ID_MOCK_REQUEST, (uint8_t []){true}, 1);
      } else {
        send_response(ID_MOCK_REQUEST, (uint8_t []){false}, 1);
      }
    }
    break;

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
    send_response(ID_GET_CALIBRATION, (uint8_t *)&settings->accel_cal, sizeof(Calibration_t) * 4);
    break;

  case ID_CANCEL_REQUEST:
    if (sys_manager_send_cmd != NULL) {
      if (sys_manager_send_cmd(ID_CANCEL_REQUEST)) {
        send_response(ID_CANCEL_REQUEST, (uint8_t []){true}, 1);
      } else {
        send_response(ID_CANCEL_REQUEST, (uint8_t []){false}, 1);
      }
    }
    break;

  default:
    return;
  }
}
