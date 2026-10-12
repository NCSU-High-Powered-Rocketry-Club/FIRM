#include "fatfs_sd.h"

#include "fatfs.h"

extern SD_HandleTypeDef hsd;

static FATFS logger_fs;
static FIL logger_file;
static DMA_HandleTypeDef *logger_dma_sdio_tx = NULL;
static bool logger_fs_ready = false;
static bool logger_file_open = false;
static bool logger_write_failed = false;

static int wait_for_write(void) {
  uint32_t start = HAL_GetTick();
  while (!fatfs_sd_is_write_ready()) {
    if (logger_write_failed || HAL_SD_GetError(&hsd) != HAL_SD_ERROR_NONE ||
        HAL_GetTick() - start >= 30000U)
      return 1;
    osDelay(1);
  }
  return 0;
}

int fatfs_sd_init(DMA_HandleTypeDef *dma_sdio_tx_handle) {
  if (dma_sdio_tx_handle == NULL)
    return 1;

  logger_dma_sdio_tx = dma_sdio_tx_handle;

  if (BSP_SD_Init() != MSD_OK)
    return 1;

  if (FATFS_UnLinkDriver(SDPath) != 0)
    return 1;
  if (FATFS_LinkDriver(&SD_Driver, SDPath) != 0)
    return 1;

  FRESULT fr = f_mount(&logger_fs, SDPath, 0);
  if (fr != FR_OK) {
    f_mount(NULL, SDPath, 0);
    logger_fs_ready = false;
    return 1;
  }

  logger_fs_ready = true;
  return 0;
}

bool fatfs_sd_file_exists(const char *filename) {
  if (!logger_fs_ready || filename == NULL)
    return false;

  FILINFO file_info;
  return f_stat(filename, &file_info) == FR_OK;
}

int fatfs_sd_create_file(const char *filename, uint64_t size_bytes) {
  if (!logger_fs_ready)
    return 1;
  if (filename == NULL)
    return 1;

  if (logger_file_open) {
    if (fatfs_sd_close())
      return 1;
  }

  FRESULT fr = f_open(&logger_file, filename, FA_CREATE_NEW | FA_WRITE);
  if (fr != FR_OK)
    return 1;

  fr = f_truncate(&logger_file);
  if (fr != FR_OK) {
    f_close(&logger_file);
    return 1;
  }

  fr = f_expand(&logger_file, (FSIZE_t)size_bytes, 1);
  if (fr != FR_OK) {
    f_close(&logger_file);
    return 1;
  }

  fr = f_sync(&logger_file);
  if (fr != FR_OK) {
    f_close(&logger_file);
    return 1;
  }

  logger_write_failed = false;
  logger_file_open = true;
  return 0;
}

bool fatfs_sd_is_write_ready(void) {
  if (!logger_file_open || logger_dma_sdio_tx == NULL || logger_write_failed ||
      HAL_DMA_GetState(logger_dma_sdio_tx) != HAL_DMA_STATE_READY ||
      HAL_SD_GetState(&hsd) != HAL_SD_STATE_READY)
    return false;
  if (HAL_SD_GetError(&hsd) != HAL_SD_ERROR_NONE) {
    logger_write_failed = true;
    return false;
  }
  if (BSP_SD_GetCardState() != SD_TRANSFER_OK)
    return false;
  return true;
}

int fatfs_sd_write_sector(const uint8_t *buffer, size_t len) {
  if (!logger_file_open || buffer == NULL || len == 0 ||
      ((uintptr_t)buffer & 3U) || len % _MAX_SS != 0 ||
      f_tell(&logger_file) % _MAX_SS != 0 ||
      len > f_size(&logger_file) - f_tell(&logger_file))
    return 1;
  if (!fatfs_sd_is_write_ready())
    return 1;

  UINT bytes_written = 0;
  sd_FastWriteFlag = 1;
  FRESULT fr = f_write(&logger_file, buffer, len, &bytes_written);
  sd_FastWriteFlag = 0;
  if (fr != FR_OK || bytes_written != len) {
    logger_write_failed = true;
    return 1;
  }
  return 0;
}

int fatfs_sd_sync(void) {
  if (!logger_file_open || wait_for_write())
    return 1;
  return f_sync(&logger_file) == FR_OK ? 0 : 1;
}

int fatfs_sd_close(void) {
  if (!logger_file_open)
    return 0;
  if (wait_for_write())
    return 1;

  FRESULT fr = f_close(&logger_file);
  if (fr != FR_OK)
    return 1;

  logger_file_open = false;
  return 0;
}
