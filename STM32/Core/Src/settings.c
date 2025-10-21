#include "settings.h"

FIRMSettings_t FIRMSettings;

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
    memcpy(&FIRMSettings, buf, sizeof(FIRMSettings_t));

    /* Optional: Basic validation (extend as needed) */
    if (FIRMSettings.checksum != 0xA5A5A5A5) {
        serialPrintStr("Settings checksum failed, device needs to be configured");
        return 1;
    }
    return 0;
}

static void settings_write_defaults(void) {
    // TODO: determine settings to use
    FIRMSettings.accel_enabled = true;
    FIRMSettings.checksum = 0xA5A5A5A5;

    // Erase sector 0 first (4 KB, covers our 1024 bytes)
    w25q128jv_erase_sector(0);

    // Write the 1024-byte block
    uint8_t buf[1024];
    memcpy(buf, &FIRMSettings, sizeof(FIRMSettings_t));
    w25q128jv_write_sector(buf, 0, 0, 1024);
}