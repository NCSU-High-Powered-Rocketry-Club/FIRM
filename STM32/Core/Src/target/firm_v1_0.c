#include "firm_v1_0.h"
#include "adxl371.h"
#include "bmp581.h"
#include "fatfs_sd.h"
#include "icm45686.h"
#include "led.h"
#include "logger_storage.h"
#include "mmc5983ma.h"
#include "settings_storage.h"

#if FIRM_HARDWARE_VERSION == VERSION_V1_0

#define FLASH_SETTINGS_SECTOR 0

#define LOGGER_SECTOR_SIZE 8192

extern DMA_HandleTypeDef hdma_sdio_tx;

static uint8_t logger_buffer_a[LOGGER_SECTOR_SIZE] = {0};
static uint8_t logger_buffer_b[LOGGER_SECTOR_SIZE] = {0};

static bool logger_target_file_exists(const char *filename) {
  if (filename == NULL)
    return false;
  return fatfs_sd_file_exists(filename);
}

static int logger_target_create_file(const char *filename, uint64_t size_bytes) {
  return fatfs_sd_create_file(filename, size_bytes);
}

static bool logger_target_write_ready(void) { return fatfs_sd_is_write_ready(); }

static int logger_target_write_sector(const uint8_t *buffer, size_t len) {
  if (buffer == NULL || len != LOGGER_SECTOR_SIZE)
    return 1;
  return fatfs_sd_write_sector(buffer, len);
}

static int settings_storage_read(uint8_t *buffer, size_t len) {
  w25q128jv_read_sector(buffer, FLASH_SETTINGS_SECTOR, 0, len);
  return 0;
}

static int settings_storage_write(uint8_t *buffer, size_t len) {
  w25q128jv_erase_sector(FLASH_SETTINGS_SECTOR);
  w25q128jv_write_sector(buffer, FLASH_SETTINGS_SECTOR, 0, len);
  return 0;
}

static uint64_t settings_read_uid(void) {
  uint64_t uid;
  w25q128jv_read_UID((uint8_t *)&uid, 8);
  return uid;
}

// Must finish before the first FreeRTOS call that allocates or enters a critical section. Until
// osKernelStart() that leaves BASEPRI raised (nesting starts at 0xaaaaaaaa), masking the
// interrupts we use, including the TIM2 HAL tick. Then HAL_Delay hangs, HAL timeouts never
// expire, and interrupt- or DMA-driven HAL calls never complete.
int firm_init_hardware(void) {
  // MX_GPIO_Init leaves the chip selects low. Deselect every SPI device before using a bus.
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // BMP581 CS pin
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // ICM45686 CS pin
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); // flash chip CS pin
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // MMC5983MA CS pin
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); // ADXL371 CS pin

  // set up settings storage (w25q128jv NOR Flash)
  SettingsStorageInterface_t settings_storage_interface = {
      .read_settings = settings_storage_read,
      .write_settings = settings_storage_write,
      .read_uid = settings_read_uid,
  };
  w25q128jv_set_spi_settings(&hspi1, GPIOC, GPIO_PIN_4);
  w25q128jv_init();
  if (settings_storage_init(&settings_storage_interface)) {
    led_set_status(FLASH_CHIP_FAIL);
    return 1;
  }

  // initialize sensor communication
  set_spi_adxl(&hspi2, GPIOA, GPIO_PIN_8);
  set_spi_bmp(&hspi2, GPIOC, GPIO_PIN_2);
  set_spi_icm(&hspi2, GPIOB, GPIO_PIN_9);
  set_spi_mmc(&hspi2, GPIOC, GPIO_PIN_7);

  // let the sensors power up before talking to them
  HAL_Delay(100);

  if (icm45686_init()) {
    led_set_status(IMU_FAIL);
    return 1;
  }

  if (mmc5983ma_init()) {
    led_set_status(MMC5983MA_FAIL);
    return 1;
  }

  if (bmp581_init()) {
    led_set_status(BMP581_FAIL);
    return 1;
  }

  if (adxl371_init()) {
    led_set_status(HIGH_G_FAIL);
    return 1;
  }

  return 0;
}

// Call from a task, after osKernelStart(). HAL_SD_Init needs the HAL tick (HAL_Delay), f_mount
// creates an RTOS object, and the first volume access (f_stat/f_open) needs the kernel running.
int firm_init_log_storage(void) {
  if (fatfs_sd_init(&hdma_sdio_tx)) {
    led_set_status(SD_FAIL);
    return 1;
  }

  // initialize logger storage interface (FatFS + SD DMA)
  LoggerStorageInterface_t logger_storage_interface = {
      .file_exists = logger_target_file_exists,
      .create_file = logger_target_create_file,
      .is_write_ready = logger_target_write_ready,
      .write_sector = logger_target_write_sector,
      .active_buffer = logger_buffer_a,
      .standby_buffer = logger_buffer_b,
      .buffer_size = LOGGER_SECTOR_SIZE,
  };
  if (logger_storage_init(&logger_storage_interface)) {
    led_set_status(SD_FAIL);
    return 1;
  }

  return 0;
}

#endif
