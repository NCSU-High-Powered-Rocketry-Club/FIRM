#include "settings.h"

FIRMSettings_t FIRMSettings;

int settings_init(SPI_HandleTypeDef* flash_hspi, GPIO_TypeDef* flash_cs_channel, uint16_t flash_cs_pin) {
    if (w25q128jv_init(flash_hspi, flash_cs_channel, flash_cs_pin)) {
        return 1;
    }
    return 0;
}