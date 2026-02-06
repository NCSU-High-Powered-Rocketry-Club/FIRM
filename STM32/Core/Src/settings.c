#include "settings.h"
#include "utils.h"
#include <stdint.h>
#include <string.h>

FIRMSettings_t firmSettings;
CalibrationSettings_t calibrationSettings;

static void settings_write_defaults(void);
static bool settings_write_flash_block(uint8_t *block_to_write);

static bool settings_write_flash_block_to_sector(uint8_t *block_to_write, uint8_t sector);

int settings_init(SPI_HandleTypeDef *flash_hspi, GPIO_TypeDef *flash_cs_channel, uint16_t flash_cs_pin) {
  // set up flash chip porting layer
  w25q128jv_set_spi_settings(flash_hspi, flash_cs_channel, flash_cs_pin);

  if (w25q128jv_init() != 1) {
    return 1; // error
  }
  // if this pin is pulled to ground (default pulled high), write settings to flash chip
  GPIO_TypeDef *write_check_pin_channel = GPIOA;
  uint16_t write_check_pin = GPIO_PIN_4;

  if (HAL_GPIO_ReadPin(write_check_pin_channel, write_check_pin) == GPIO_PIN_RESET) {
    settings_write_defaults();
  }

  // read settings block
  uint8_t buf[SETTINGS_FLASH_BLOCK_SIZE_BYTES];
  w25q128jv_read_sector(buf, 0, 0, SETTINGS_FLASH_BLOCK_SIZE_BYTES);
  memcpy(&calibrationSettings, buf, sizeof(CalibrationSettings_t));
  memcpy(&firmSettings, buf + sizeof(CalibrationSettings_t), sizeof(FIRMSettings_t));

  // uid validation
  uint64_t uid;
  w25q128jv_read_UID((uint8_t *)&uid, 8);
  if (uid != firmSettings.device_uid) {
    serialPrintStr("Settings initialization failed, device may need to be configured");
    return 1;
  }
  return 0;
}

bool settings_write_calibration_settings(AccelCalibration_t *accel_cal_settings, GyroCalibration_t *gyro_cal_settings, MagCalibration_t *mag_cal_settings) {
  if (accel_cal_settings == NULL && gyro_cal_settings == NULL && mag_cal_settings == NULL) {
    return false;
  }

  uint8_t buffer_to_write[SETTINGS_FLASH_BLOCK_SIZE_BYTES];
  // Reads the current settings (FIRM and Calibration) into buffer_to_write
  w25q128jv_read_sector(buffer_to_write, 0, 0, SETTINGS_FLASH_BLOCK_SIZE_BYTES);

  // Writes over just the calibration settings portion
  if (accel_cal_settings != NULL)
    memcpy(buffer_to_write, accel_cal_settings, sizeof(AccelCalibration_t));
  if (gyro_cal_settings != NULL)
    memcpy(buffer_to_write + sizeof(AccelCalibration_t), gyro_cal_settings, sizeof(GyroCalibration_t));
  if (mag_cal_settings != NULL)
    memcpy(buffer_to_write + sizeof(AccelCalibration_t) + sizeof(GyroCalibration_t), mag_cal_settings, sizeof(MagCalibration_t));
  bool ok = settings_write_flash_block(buffer_to_write);
  if (ok) {
    memcpy(&calibrationSettings.icm45686_accel, buffer_to_write, sizeof(AccelCalibration_t));
    memcpy(&calibrationSettings, buffer_to_write + sizeof(AccelCalibration_t), sizeof(GyroCalibration_t));
    memcpy(&calibrationSettings, buffer_to_write + sizeof(AccelCalibration_t) + sizeof(GyroCalibration_t), sizeof(MagCalibration_t));
  }
  return 1;
}

bool settings_write_firm_settings(FIRMSettings_t *firm_settings) {
  if (firm_settings == NULL) {
    return false;
  }

  uint8_t buffer_to_write[SETTINGS_FLASH_BLOCK_SIZE_BYTES];
  // Reads the current settings (FIRM and Calibration) into buffer_to_write
  w25q128jv_read_sector(buffer_to_write, 0, 0, SETTINGS_FLASH_BLOCK_SIZE_BYTES);

  // Writes over just the FIRM settings portion (after the calibration settings)
  memcpy(&buffer_to_write[sizeof(CalibrationSettings_t)], firm_settings, sizeof(FIRMSettings_t));
  bool ok = settings_write_flash_block(buffer_to_write);
  if (ok) {
    firmSettings = *firm_settings;
  }
  return ok;
}

static void settings_write_defaults(void) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i != j) {
        calibrationSettings.icm45686_accel.scale_multiplier[3 * i + j] = 0.0F;
        calibrationSettings.icm45686_gyro.scale_multiplier[3 * i + j] = 0.0F;
        calibrationSettings.mmc5983ma_mag.scale_multiplier[3 * i + j] = 0.0F;
        continue;
      }
      calibrationSettings.icm45686_accel.scale_multiplier[3 * i + j] = 1.0F;
      calibrationSettings.icm45686_gyro.scale_multiplier[3 * i + j] = 1.0F;
      calibrationSettings.mmc5983ma_mag.scale_multiplier[3 * i + j] = 1.0F;
    }
    calibrationSettings.icm45686_accel.offset_gs[i] = 0.0F;
    calibrationSettings.icm45686_gyro.offset_dps[i] = 0.0F;
    calibrationSettings.mmc5983ma_mag.offset_ut[i] = 0.0F;
  }
  
  // TODO: determine settings to use
  w25q128jv_read_UID((uint8_t *)&firmSettings.device_uid, 8);
  firmSettings.usb_transfer_enabled = true;
  firmSettings.uart_transfer_enabled = false;
  firmSettings.i2c_transfer_enabled = false;
  firmSettings.spi_transfer_enabled = false;
  strcpy(firmSettings.device_name, "FIRM Device");
  strcpy(firmSettings.firmware_version, "v1.0.0");
  firmSettings.frequency_hz = 100;

  (void)settings_write_calibration_settings(&calibrationSettings.icm45686_accel, &calibrationSettings.icm45686_gyro, &calibrationSettings.mmc5983ma_mag);
  (void)settings_write_firm_settings(&firmSettings);
}

static bool settings_write_flash_block(uint8_t *block_to_write) {
  if (block_to_write == NULL) {
    return false;
  }

  // Erase sector 0 first (4 KB, covers our 1024 bytes)
  w25q128jv_erase_sector(0);
  w25q128jv_write_sector(block_to_write, 0, 0, SETTINGS_FLASH_BLOCK_SIZE_BYTES);

  // verify
  // Read back in small chunks to avoid large stack allocations.
  uint8_t verify[64];
  for (uint32_t offset = 0; offset < SETTINGS_FLASH_BLOCK_SIZE_BYTES; offset += (uint32_t)sizeof(verify)) {
    uint32_t remaining = SETTINGS_FLASH_BLOCK_SIZE_BYTES - offset;
    uint32_t to_read = remaining < (uint32_t)sizeof(verify) ? remaining : (uint32_t)sizeof(verify);

    w25q128jv_read_sector(verify, 0, offset, to_read);
    if (memcmp(verify, block_to_write + offset, to_read) != 0) {
      return false;
    }
  }
  return true;
}

static bool settings_write_flash_block_to_sector(uint8_t *block_to_write, uint8_t sector) {
  if (block_to_write == NULL) {
    return false;
  }

  // Erase the specified sector first (4 KB, covers our 512 bytes)
  w25q128jv_erase_sector(sector);
  w25q128jv_write_sector(block_to_write, sector, 0, SETTINGS_FLASH_BLOCK_SIZE_BYTES);

  // verify
  // Read back in small chunks to avoid large stack allocations.
  uint8_t verify[64];
  for (uint32_t offset = 0; offset < SETTINGS_FLASH_BLOCK_SIZE_BYTES; offset += (uint32_t)sizeof(verify)) {
    uint32_t remaining = SETTINGS_FLASH_BLOCK_SIZE_BYTES - offset;
    uint32_t to_read = remaining < (uint32_t)sizeof(verify) ? remaining : (uint32_t)sizeof(verify);

    w25q128jv_read_sector(verify, sector, offset, to_read);
    if (memcmp(verify, block_to_write + offset, to_read) != 0) {
      return false;
    }
  }
  return true;
}

bool settings_write_mock_settings(FIRMSettings_t *firm_settings, CalibrationSettings_t *calibration_settings) {
  if (firm_settings == NULL || calibration_settings == NULL) {
    return false;
  }

  uint8_t buffer_to_write[SETTINGS_FLASH_BLOCK_SIZE_BYTES];

  // Write calibration settings at the beginning
  memcpy(buffer_to_write, calibration_settings, sizeof(CalibrationSettings_t));
  memcpy(&calibrationSettings, calibration_settings, sizeof(CalibrationSettings_t));

  // Write firm settings after calibration settings
  memcpy(buffer_to_write + sizeof(CalibrationSettings_t), firm_settings, sizeof(FIRMSettings_t));

  // Write to sector 2 instead of sector 0
  return settings_write_flash_block_to_sector(buffer_to_write, 2);
}
