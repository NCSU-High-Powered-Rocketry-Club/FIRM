#include "fatfs_sd.h"

#include "fatfs.h"

static FATFS logger_fs;
static FIL logger_file;
static DMA_HandleTypeDef *logger_dma_sdio_tx = NULL;
static bool logger_fs_ready = false;
static bool logger_file_open = false;

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
    f_close(&logger_file);
    logger_file_open = false;
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

  logger_file_open = true;
  return 0;
}

bool fatfs_sd_is_write_ready(void) {
  if (logger_dma_sdio_tx == NULL)
    return false;
  return HAL_DMA_GetState(logger_dma_sdio_tx) == HAL_DMA_STATE_READY;
}

int fatfs_sd_write_sector(const uint8_t *buffer, size_t len) {
  if (!logger_file_open)
    return 1;
  if (buffer == NULL || len == 0)
    return 1;
  if (!fatfs_sd_is_write_ready())
    return 1;

  UINT bytes_written = 0;
  sd_FastWriteFlag = 1;
  FRESULT fr = f_write(&logger_file, buffer, len, &bytes_written);
  sd_FastWriteFlag = 0;

  if (fr != FR_OK || bytes_written != len)
    return 1;

  return 0;
}

int fatfs_sd_sync(void) {
  if (!logger_file_open)
    return 1;
  return f_sync(&logger_file) == FR_OK ? 0 : 1;
}

int fatfs_sd_close(void) {
  if (!logger_file_open)
    return 0;

  FRESULT fr = f_close(&logger_file);
  if (fr != FR_OK)
    return 1;

  logger_file_open = false;
  return 0;
}
