#include "settings_manager.h"
#include "system_settings.h"

#define SETTINGS_WRITE_DEFAULT 0

#if SETTINGS_WRITE_DEFAULT == 1
static void settings_write_defaults(SystemSettings_t *settings);
#endif

static SystemSettings_t FIRMSystemSettings = {0};

int settings_manager_init(void) {
  #if SETTINGS_WrITE_DEFAULT == 1
  settings_write_defaults(&FIRMSystemSettings);
  #endif
  settings_read_from_storage(&FIRMSystemSettings);

  // uid validation
  uint64_t uid = settings_read_storage_uid();
  if (uid != FIRMSystemSettings.device_uid) {
    // Settings initialization failed, device may need to be configured
    return 1;
  }
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

#if SETTINGS_WRITE_DEFAULT == 1
static void settings_write_defaults(SystemSettings_t *settings) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i != j) {
        settings->accel_cal.scale_matrix[3 * i + j] = 0.0F;
        settings->gyro_cal.scale_matrix[3 * i + j] = 0.0F;
        settings->mag_cal.scale_matrix[3 * i + j] = 0.0F;
        settings->high_g_cal.scale_matrix[3 * i + j] = 0.0F;
        continue;
      }
      settings->accel_cal.scale_matrix[3 * i + j] = 1.0F;
      settings->gyro_cal.scale_matrix[3 * i + j] = 1.0F;
      settings->mag_cal.scale_matrix[3 * i + j] = 1.0F;
      settings->high_g_cal.scale_matrix[3 * i + j] = 1.0F;
    }
    settings->accel_cal.offset_gs[i] = 0.0F;
    settings->gyro_cal.offset_dps[i] = 0.0F;
    settings->mag_cal.offset_ut[i] = 0.0F;
    settings->high_g_cal.offset_gs[i] = 0.0F;
  }

  settings->device_uid = settings_read_storage_uid();
  settings->.usb_transfer_enabled = true;
  settings->.uart_transfer_enabled = false;
  settings->.i2c_transfer_enabled = false;
  settings->.spi_transfer_enabled = false;
  strcpy(settings->.device_name, "FIRM Device");
  strcpy(settings->.firmware_version, FIRM_FIRMWARE_VERSION);
  settings->.frequency_hz = 100;

  (void)settings_write_calibration_settings(
      &settings->accel_cal, &settings->gyro_cal,
      &settings->mag_cal, &settings->high_g_cal);
  (void)settings_write_firm_settings(&settings->);
}
#endif