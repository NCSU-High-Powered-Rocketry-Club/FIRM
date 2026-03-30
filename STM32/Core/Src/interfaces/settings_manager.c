#include "settings_manager.h"
#include "system_settings.h"

#define FIRM_DEFAULT_DEVICE_NAME "FIRM Device"
#define SETTINGS_WRITE_DEFAULT 0

#if SETTINGS_WRITE_DEFAULT == 1
static bool settings_write_defaults(void);
#endif

static SystemSettings_t FIRMSystemSettings = {0};

static void settings_set_firmware_version(void);

int settings_manager_init(void) {
  #if SETTINGS_WRITE_DEFAULT == 1
  if (!settings_write_defaults())
    return 1;
  #endif
  settings_read_from_storage(&FIRMSystemSettings);

  // uid validation
  uint64_t uid = settings_read_storage_uid();
  if (uid != FIRMSystemSettings.device_uid) {
    // Settings initialization failed, device may need to be configured
    return 1;
  }
  // firmware version overwrite
  settings_set_firmware_version();
  return 0;
}

bool settings_write_calibration(Calibration_t *accel_calibraion,
                                Calibration_t *gyro_calibration, Calibration_t *mag_calibration,
                                Calibration_t *high_g_calibration) {
  if (accel_calibraion == NULL && gyro_calibration == NULL && mag_calibration == NULL &&
      high_g_calibration == NULL) {
    return false;
  }

  // Sets only the calibration fields that are provided
  if (accel_calibraion != NULL) {
    FIRMSystemSettings.accel_cal = *accel_calibraion;
  }
  if (gyro_calibration != NULL) {
    FIRMSystemSettings.gyro_cal = *gyro_calibration;
  }
  if (mag_calibration != NULL) {
    FIRMSystemSettings.mag_cal = *mag_calibration;
  }
  if (high_g_calibration != NULL) {
    FIRMSystemSettings.high_g_cal = *high_g_calibration;
  }
  settings_write_to_storage(&FIRMSystemSettings);
  return true;
}

bool settings_write_firm_settings(SystemSettings_t *settings) {
  if (settings == NULL)
    return false;
  FIRMSystemSettings = *settings;
  settings_write_to_storage(&FIRMSystemSettings);
  return true;
}

const SystemSettings_t *get_settings(void) {
  return (const SystemSettings_t*)&FIRMSystemSettings;
}

static void settings_set_firmware_version(void) {
  strcpy(FIRMSystemSettings.firmware_version, FIRM_FIRMWARE_VERSION);
  settings_write_firm_settings(&FIRMSystemSettings);
}

#if SETTINGS_WRITE_DEFAULT == 1
static bool settings_write_defaults(void) {
  SystemSettings_t default_settings = {0};

  // default calibration is 0.0 offset, and scale factor matrix is the identity matrix
  for (int i = 0; i < 3; i++) {
    default_settings.accel_cal.scale_matrix[3 * i + i] = 1.0F;
    default_settings.gyro_cal.scale_matrix[3 * i + i] = 1.0F;
    default_settings.mag_cal.scale_matrix[3 * i + i] = 1.0F;
    default_settings.high_g_cal.scale_matrix[3 * i + i] = 1.0F;
  }
  
  default_settings.device_uid = settings_read_storage_uid();
  default_settings.usb_transfer_enabled = true;
  default_settings.uart_transfer_enabled = false;
  default_settings.i2c_transfer_enabled = false;
  default_settings.spi_transfer_enabled = false;
  strcpy(default_settings.device_name, FIRM_DEFAULT_DEVICE_NAME);
  strcpy(default_settings.firmware_version, FIRM_FIRMWARE_VERSION);
  default_settings.frequency_hz = 100;

  return settings_write_firm_settings(&default_settings);
}
#endif