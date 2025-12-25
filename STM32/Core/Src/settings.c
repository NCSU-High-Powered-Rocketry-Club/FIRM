#include "settings.h"

#include <string.h>

FIRMSettings_t firmSettings;
CalibrationSettings_t calibrationSettings;

static void settings_write_defaults(void);
static void settings_write_flash_block(uint8_t* block_to_write);

int settings_init(SPI_HandleTypeDef* flash_hspi, GPIO_TypeDef* flash_cs_channel, uint16_t flash_cs_pin) {
    // set up flash chip porting layer
    w25q128jv_set_spi_settings(flash_hspi, flash_cs_channel, flash_cs_pin);
    
    if (w25q128jv_init() != 1) {
        return 1; // error
    }
    // if this pin is pulled to ground (default pulled high), write settings to flash chip
    GPIO_TypeDef* write_check_pin_channel = GPIOA;
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

void settings_write_calibration_settings(CalibrationSettings_t* calibration_settings) {
    uint8_t buf[SETTINGS_FLASH_BLOCK_SIZE_BYTES];
    w25q128jv_read_sector(buf, 0, 0, SETTINGS_FLASH_BLOCK_SIZE_BYTES);

    memcpy(buf, calibration_settings, sizeof(CalibrationSettings_t));
    settings_write_flash_block(buf);

    memcpy(&calibrationSettings, calibration_settings, sizeof(CalibrationSettings_t));
}

void settings_write_firm_settings(FIRMSettings_t* firm_settings) {
    uint8_t buf[SETTINGS_FLASH_BLOCK_SIZE_BYTES];
    w25q128jv_read_sector(buf, 0, 0, SETTINGS_FLASH_BLOCK_SIZE_BYTES);

    FIRMSettings_t sanitized = *firm_settings;
    sanitized.device_name[sizeof(sanitized.device_name) - 1] = '\0';
    sanitized.firmware_version[sizeof(sanitized.firmware_version) - 1] = '\0';

    memcpy(buf + sizeof(CalibrationSettings_t), &sanitized, sizeof(FIRMSettings_t));
    settings_write_flash_block(buf);

    firmSettings = sanitized;
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
    w25q128jv_read_UID((uint8_t*)&firmSettings.device_uid, 8);
    firmSettings.usb_transfer_enabled = true;
    firmSettings.uart_transfer_enabled = true;
    firmSettings.i2c_transfer_enabled = true;
    firmSettings.spi_transfer_enabled = true;
    strcpy(firmSettings.device_name, "FIRM Device");
    strcpy(firmSettings.firmware_version, "v1.0.0");
    firmSettings.frequency_hz = 100;

    settings_write_calibration_settings(&calibrationSettings);
    settings_write_firm_settings(&firmSettings);
}

static void settings_write_flash_block(uint8_t* block_to_write) {
    // Erase sector 0 first (4 KB, covers our 1024 bytes)
    w25q128jv_erase_sector(0);
    w25q128jv_write_sector(block_to_write, 0, 0, SETTINGS_FLASH_BLOCK_SIZE_BYTES);
}
