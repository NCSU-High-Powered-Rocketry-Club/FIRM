/*
 * logger.h
 *
 *  Created on: Sep 19, 2025
 *      Author: craig
 */

#pragma once

#include "settings_manager.h"
#include <stdint.h>
#include <string.h>

#include "fatfs.h"
#include "ff.h"

/** The current version of the FIRM log file */
#define FIRM_LOG_HEADER_TEXT "FIRM LOG v1.3\n"

/** The sector size of the SD card. This is the smallest we are able to use. */
#define SD_SECTOR_SIZE 8192

/**
 * @brief scale factor values for the sensors to include in the header file
 */
typedef struct {
  float temp_sf;
  float pressure_sf;
  float accel_sf;
  float angular_rate_sf;
  float magnetic_field_sf;
  float hi_g_accel_sf;
} SensorScaleFactors_t;

/**
 * @brief Initializes the SD card and creates the log file.
 *
 * @param dma_sdio_tx_handle the handle for the DMA SDIO
 * @retval File Status error code, 0 on success.
 */
FRESULT logger_init(DMA_HandleTypeDef *dma_sdio_tx_handle);

/**
 * @brief writes an initial header line to the Micro SD Card file
 *
 * @param sensor_scale_factors address of the struct with the scale factor values of the sensors
 * @retval File Status error code, 0 on success.
 */
FRESULT logger_write_header(SensorScaleFactors_t *sensor_scale_factors);

/**
 * @brief Appends a mock header to the current log file
 *
 * @param firm_settings pointer to the mock settings
 * @param sensor_scale_factors address of the struct with the scale factor values from the mock
 * header
 * @retval File Status error code, 0 on success.
 */
FRESULT logger_append_mock_header(SystemSettings_t *settings, SensorScaleFactors_t *sensor_scale_factors);

/**
 * @brief allocates space for a sensor packet in the logger, not including the metadata info.
 * @note also ensures there is enough space available in the buffer. Swaps buffer if necessary.
 *
 * @param capacity the size of the packet in bytes to allocate size for
 * @retval void pointer to the allocated memory, or NULL if not enough memory
 */
void *logger_malloc_packet(size_t capacity);

/**
 * @brief writes the packet to the logger buffer, and adds the metadata
 * @note advances the index of the buffer to prepare for next write
 *
 * @param type the identifier character of the packet
 * @param packet_size the size of the packet to add to the buffer
 * @retval None
 */
void logger_write_entry(char type, size_t packet_size);
