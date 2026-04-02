/*
 * logger.c
 *
 *  Created on: Sep 19, 2025
 *      Author: craig
 */

#include "logger.h"
#include "settings_manager.h"
#include <stdint.h>
#include <stdio.h>

#include <string.h>

/**
 * @brief Ensure that there is enough space available in the current buffer.
 * @note Swaps buffers if necessary, when other buffer is full
 *
 * @param capacity the number of bytes to ensure capacity of
 * @retval File Status error code, 0 on success.
 */
static FRESULT logger_ensure_capacity(size_t capacity);

/**
 * @brief writes a filled buffer to the Micro SD Card via DMA
 *
 * @retval File Status error code, 0 on success.
 */
static FRESULT logger_write();

/**
 * @brief swaps the buffers for the DMA logging when one gets full
 */
static void logger_swap_buffers();

/**
 * This is the size in bytes of the type of packet and the timestamp associated
 * with it.
 */
static const int packet_timestamp_size = 4;

static char buffer0[SD_SECTOR_SIZE];
static char buffer1[SD_SECTOR_SIZE];
static char *current_buffer = buffer0;
static size_t current_offset = 0;
static FATFS fs;
static FIL log_file;
static DMA_HandleTypeDef *hdma_sdio_tx; // Link to the DMA handler to check if busy

TCHAR file_name[32] = {'\0'};

FRESULT logger_init(DMA_HandleTypeDef *dma_sdio_tx_handle) {
  // set the dma handle to the static variable
  if (dma_sdio_tx_handle == NULL) {
    return FR_DENIED;
  }
  hdma_sdio_tx = dma_sdio_tx_handle;

  // Re-initialize SD
  if (BSP_SD_Init() != MSD_OK) {
    return FR_NOT_READY;
  }

  // Re-initialize FATFS
  if (FATFS_UnLinkDriver(SDPath) != 0) {
    return FR_NOT_READY;
  }
  if (FATFS_LinkDriver(&SD_Driver, SDPath) != 0) {
    return FR_NOT_READY;
  }

  // Mount file system
  FRESULT fr = f_mount(&fs, SDPath, 0);
  if (fr != FR_OK) {
    f_mount(NULL, SDPath, 0);
    return fr;
  }

  uint8_t file_index = 0;
  DIR dj;
  FILINFO fno;

  // Find the next available log file
  do {
    file_index++;
    sprintf(file_name, "log%i.frm", file_index);
    fr = f_findfirst(&dj, &fno, "/", file_name);
  } while (fr == FR_OK && fno.fname[0]);

  if (fr != FR_OK) {
    return fr;
  }

  // Open the file
  fr = f_open(&log_file, file_name, FA_CREATE_NEW | FA_WRITE);
  if (fr != FR_OK) {
    f_mount(0, SDPath, 0);
    return fr;
  }

  // Allocate a contiguous area to the file
  // We need to do this to avoid f_sync (slow!)
  {
    fr = f_truncate(&log_file);
    HAL_Delay(10);
    if (fr != FR_OK) {
      return fr;
    }

    // 2e8 bytes = (1 hour * ((8192 bytes * 4.4hz) * 1.5))
    fr = f_expand(&log_file, (FSIZE_t)2e8, 1);
    if (fr != FR_OK) {
      return fr;
    }
  }
  f_sync(&log_file);

  return fr;
}

FRESULT logger_write_header(SensorScaleFactors_t *sensor_scale_factors) {
  const char *firm_log_header = FIRM_LOG_HEADER_TEXT;
  size_t header_len = strlen(firm_log_header);
  size_t scale_factor_len = sizeof(SensorScaleFactors_t);
  size_t system_settings_len = sizeof(SystemSettings_t);

  FRESULT error_status = logger_ensure_capacity(header_len + scale_factor_len + system_settings_len);
  if (error_status) {
    return error_status;
  }

  // copy "FIRM LOG" text
  // NOLINTNEXTLINE(bugprone-not-null-terminated-result)
  memcpy(current_buffer + current_offset, firm_log_header, header_len);
  current_offset += header_len;
  // copy in system settings
  const SystemSettings_t *system_settings = get_settings();
  memcpy(current_buffer + current_offset, system_settings, system_settings_len);
  current_offset += system_settings_len;
  // copy sensor scale factor struct
  memcpy(current_buffer + current_offset, sensor_scale_factors, scale_factor_len);
  current_offset += scale_factor_len;

  return error_status;
}

void *logger_malloc_packet(size_t capacity) {
  if (logger_ensure_capacity(capacity)) {
    return NULL;
  }
  return &current_buffer[current_offset];
}

void logger_write_entry(char type, size_t packet_size) {
  // the packet has a 4 byte timestamp, but we want to replace the most significant byte
  // and replace with identification letter
  current_buffer[current_offset++] = type;
  // advance current offset by timestamp size (fixed) and packet size (variable)
  current_offset += packet_timestamp_size;
  current_offset += packet_size;
}

static FRESULT logger_ensure_capacity(size_t capacity) {
  if (current_offset + capacity > SD_SECTOR_SIZE) {
    logger_write();

    logger_swap_buffers();
  }

  // TODO error handling
  return FR_OK;
}

static FRESULT logger_write() {
  if (HAL_DMA_GetState(hdma_sdio_tx) != HAL_DMA_STATE_READY) {
    // full
    return FR_DISK_ERR;
  }

  // Pad the buffer
  for (size_t i = current_offset; i < SD_SECTOR_SIZE; i++) {
    current_buffer[i] = 0;
  }

  UINT bytes_written = 0;

  // Set fast write so that it doesn't block on the dma request
  // See SD_write for fast implementation
  // We need to set it back because some of the internal SD card functions (like f_expand and
  // f_sync) need the old behavior
  sd_FastWriteFlag = 1;
  // We need to log exactly BUFFER_SIZE = sector size in order for this to work in future writes.
  FRESULT fr = f_write(&log_file, current_buffer, SD_SECTOR_SIZE, &bytes_written);
  sd_FastWriteFlag = 0;

  if (fr != FR_OK) {
    // error in logger write
    return fr;
  }

  return fr;
}

static void logger_swap_buffers() {
  if (current_buffer == buffer0) {
    current_buffer = buffer1;
  } else {
    current_buffer = buffer0;
  }

  current_offset = 0;
}

FRESULT logger_append_mock_header(SystemSettings_t *settings, SensorScaleFactors_t *sensor_scale_factors) {
  if (settings == NULL ||  sensor_scale_factors == NULL) {
    return FR_INVALID_PARAMETER;
  }

  const char *mock_header = FIRM_LOG_HEADER_TEXT;
  size_t header_len = strlen(mock_header);
  size_t system_settings_len = sizeof(SystemSettings_t);
  size_t scale_factor_len = sizeof(SensorScaleFactors_t);

  FRESULT error_status = logger_ensure_capacity(header_len + system_settings_len + scale_factor_len);
  if (error_status) {
    return error_status;
  }

  // Append mock header marker
  // NOLINTNEXTLINE(bugprone-not-null-terminated-result)
  memcpy(current_buffer + current_offset, mock_header, header_len);
  current_offset += header_len;

  // Append mock firmware settings
  memcpy(current_buffer + current_offset, settings, system_settings_len);
  current_offset += system_settings_len;

  // Append sensor scale factors
  memcpy(current_buffer + current_offset, sensor_scale_factors, scale_factor_len);
  current_offset += scale_factor_len;

  return error_status;
}
