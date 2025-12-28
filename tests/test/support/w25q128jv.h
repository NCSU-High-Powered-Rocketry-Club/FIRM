#pragma once
#include <stdint.h>
#include <stddef.h>
#include "stm32_hal_stubs.h"

void w25q128jv_set_spi_settings(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin);
int  w25q128jv_init(void);
void w25q128jv_read_sector(uint8_t* buf, uint32_t sector, uint32_t offset, uint32_t len);
void w25q128jv_write_sector(const uint8_t* buf, uint32_t sector, uint32_t offset, uint32_t len);
void w25q128jv_erase_sector(uint32_t sector);
void w25q128jv_read_UID(uint8_t* out, uint32_t len);
