#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>

void w25q128jv_set_spi_settings(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin);
/////////////////////////////////////////////////////////////////

/**
 * @brief Read JEDEC ID
 * @ref 8.1.1 Manufacturer and Device Identification
 * @ref 8.1.2 Instruction Set Table 1 (Standard SPI Instructions)
 * @return uint32_t receive order (MF7-MF0) (ID15-ID8) (ID7-ID0)
 */
uint32_t w25q128jv_read_JEDEC_ID(void);

/**
 * @brief Read Unique ID
 * @param uid pointer to buffer to store the Unique ID
 * @param length number of bytes to read
 * @return uint8_t
 *      1 - Success
 *      0 - Failed, the parameter 'bytes' is not 8.
 */
uint8_t w25q128jv_read_UID(uint8_t *uid, uint8_t length);

/**
 * @brief Initialize w25q128jv
 * @return uint8_t
 *      1 - Success
 *      0 - Failed
 */
uint8_t w25q128jv_init(void);

/**
 * @brief Erase the entire chip.
 */
void w25q128jv_erase_chip(void);

/**
 * @brief Erase a sector.
 * @param sector_num Sector number to be erased.
 */
void w25q128jv_erase_sector(uint32_t sector_num);

/**
 * @brief Erase a block.
 * @param block_num Block number to be erased.
 */
void w25q128jv_erase_block(uint32_t block_num);

/**
 * @brief Read one byte from W25Q128JV.
 * @param pbuf Pointer to the data buffer for reading.
 * @param addr Target address to read from.
 */
void w25q128jv_read_byte(uint8_t *pbuf, uint32_t addr);

/**
 * @brief Read multiple bytes from W25Q128JV..
 * @param pbuf Pointer to the data buffer for reading.
 * @param addr Target address to read from.
 * @param length Number of bytes to read.
 */
void w25q128jv_read_bytes(uint8_t *pbuf, uint32_t addr, uint32_t length);

/**
 * @brief w25q128jv read from the page number.
 * @param pbuf Pointer to the data buffer for reading.
 * @param page_num Page number to read from.
 * @param offset Offset within the page to start reading from.
 * @param length Number of bytes to read.
 */
void w25q128jv_read_page(uint8_t *pbuf,
                         uint32_t page_num,
                         uint32_t offset,
                         uint32_t length);

/**
 * @brief w25q128jv read from the sector number.
 * @param pbuf Pointer to the data buffer for reading.
 * @param sector_num Sector number to read from.
 * @param offset Offset within the sector to start reading from.
 * @param length Number of bytes to read.
 */
void w25q128jv_read_sector(uint8_t *pbuf,
                           uint32_t sector_num,
                           uint32_t offset,
                           uint32_t length);

/**
 * @brief w25q128jv read from the block number.
 * @param pbuf Pointer to the data buffer for reading.
 * @param block_num Block number to read from.
 * @param offset Offset within the block to start reading from.
 * @param length Number of bytes to read.
 */
void w25q128jv_read_block(uint8_t *pbuf,
                          uint32_t block_num,
                          uint32_t offset,
                          uint32_t length);

////////////////////////////////////////////////////////////////////////

/**
 * @brief Write a single byte to the W25Q128JV.
 * @param pbuf Pointer to the data byte to be written.
 * @param addr Address in memory where the pbuf will be written.
 */
void w25q128jv_write_byte(uint8_t pbuf, uint32_t addr);

/**
 * @brief Write data to a page on the W25Q128JV.
 * @param pbuf Pointer to the data buffer to be written.
 * @param page_num Number of the page to write to.
 * @param offset Offset within the page to start writing data.
 * @param length Length of data to be written, in bytes.
 */
void w25q128jv_write_page(uint8_t *pbuf,
                          uint32_t page_num,
                          uint32_t offset,
                          uint32_t length);

/**
 * @brief Write data to a sector on the W25Q128JV.
 * @param pbuf Pointer to the data buffer to be written.
 * @param sector_num Number of the sector to write to.
 * @param offset Offset within the sector to start writing data.
 * @param length Length of data to be written, in bytes.
 */
void w25q128jv_write_sector(uint8_t *pbuf,
                            uint32_t sector_num,
                            uint32_t offset,
                            uint32_t length);

/**
 * @brief Write data to a block on the W25Q128JV.
 * @param pbuf Pointer to the data buffer to be written.
 * @param block_num Number of the block to write to.
 * @param offset Offset within the block to start writing data.
 * @param length Length of data to be written, in bytes.
 */
void w25q128jv_write_block(uint8_t *pbuf,
                           uint32_t block_num,
                           uint32_t offset,
                           uint32_t length);