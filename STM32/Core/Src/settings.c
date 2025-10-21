#include "settings.h"

FIRMSettings_t firmSettings;
CalibrationSettings_t calibrationSettings;

static void settings_write_defaults(void);

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

    // read 1024 bytes containing settings
    uint8_t buf[1024];
    w25q128jv_read_sector(buf, 0, 0, 1024);
    memcpy(&calibrationSettings, buf, sizeof(CalibrationSettings_t));
    memcpy(&firmSettings, buf + sizeof(CalibrationSettings_t), sizeof(FIRMSettings_t));

    // checksum validation
    if (firmSettings.checksum != 0xA5A5A5A5) {
        serialPrintStr("Settings checksum failed, device may need to be configured");
        return 1;
    }
    return 0;
}

static void settings_write_defaults(void) {

    calibrationSettings.icm45686_accel.offset_gs[0] = 0.0F;
    calibrationSettings.icm45686_accel.offset_gs[1] = 0.0F;
    calibrationSettings.icm45686_accel.offset_gs[2] = 0.0F;

    calibrationSettings.icm45686_gyro.offset_dps[0] = 0.0F;
    calibrationSettings.icm45686_gyro.offset_dps[1] = 0.0F;
    calibrationSettings.icm45686_gyro.offset_dps[2] = 0.0F;

    calibrationSettings.mmc5983ma_mag.offset_ut[0] = 0.0F;
    calibrationSettings.mmc5983ma_mag.offset_ut[1] = 0.0F;
    calibrationSettings.mmc5983ma_mag.offset_ut[2] = 0.0F;

    calibrationSettings.icm45686_accel.scale_multiplier[0] = 1.0F;
    calibrationSettings.icm45686_accel.scale_multiplier[1] = 1.0F;
    calibrationSettings.icm45686_accel.scale_multiplier[2] = 1.0F;

    calibrationSettings.icm45686_gyro.scale_multiplier[0] = 1.0F;
    calibrationSettings.icm45686_gyro.scale_multiplier[1] = 1.0F;
    calibrationSettings.icm45686_gyro.scale_multiplier[2] = 1.0F;

    calibrationSettings.mmc5983ma_mag.scale_multiplier[0] = 1.0F;
    calibrationSettings.mmc5983ma_mag.scale_multiplier[1] = 1.0F;
    calibrationSettings.mmc5983ma_mag.scale_multiplier[2] = 1.0F;

    // TODO: determine settings to use
    firmSettings.checksum = 0xA5A5A5A5;

    // Erase sector 0 first (4 KB, covers our 1024 bytes)
    w25q128jv_erase_sector(0);

    // Write the 1024-byte block
    uint8_t buf[1024];
    memcpy(buf, &calibrationSettings, sizeof(CalibrationSettings_t));
    memcpy(buf + sizeof(CalibrationSettings_t), &firmSettings, sizeof(FIRMSettings_t));
    w25q128jv_write_sector(buf, 0, 0, 1024);
}