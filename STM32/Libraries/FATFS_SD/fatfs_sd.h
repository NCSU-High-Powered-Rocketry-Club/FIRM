#pragma once

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Initializes SD card + FatFS plumbing for file operations.
 * @note This wraps BSP SD init, driver link/unlink, and f_mount.
 *
 * @param dma_sdio_tx_handle DMA handle used to gate non-blocking SD writes.
 * @retval 0 on success, 1 on error.
 */
int fatfs_sd_init(DMA_HandleTypeDef *dma_sdio_tx_handle);

/**
 * @brief Checks whether a file exists on the mounted filesystem.
 *
 * @param filename Null-terminated file path/name.
 * @retval true when file metadata lookup succeeds.
 */
bool fatfs_sd_file_exists(const char *filename);

/**
 * @brief Creates a new writable file and preallocates contiguous space.
 * @note Existing open file handle managed by this module is closed first.
 *
 * @param filename Null-terminated file path/name to create.
 * @param size_bytes Number of bytes to preallocate contiguously.
 * @retval 0 on success, 1 on error.
 */
int fatfs_sd_create_file(const char *filename, uint64_t size_bytes);

/**
 * @brief Reports whether SDIO TX DMA is ready for another write.
 *
 * @retval true when DMA state is HAL_DMA_STATE_READY.
 */
bool fatfs_sd_is_write_ready(void);

/**
 * @brief Writes one sector-sized chunk to the currently open file.
 * @note Uses sd_FastWriteFlag to keep DMA-backed FatFS writes fast.
 *
 * @param buffer Data buffer to write.
 * @param len Number of bytes to write.
 * @retval 0 on success, 1 on error.
 */
int fatfs_sd_write_sector(const uint8_t *buffer, size_t len);

/**
 * @brief Flushes the currently open file metadata/data to media.
 *
 * @retval 0 on success, 1 on error.
 */
int fatfs_sd_sync(void);

/**
 * @brief Closes the currently open file, if any.
 *
 * @retval 0 on success, 1 on error.
 */
int fatfs_sd_close(void);
