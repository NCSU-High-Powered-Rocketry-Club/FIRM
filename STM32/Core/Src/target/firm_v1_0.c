#include "firm_v1_0.h"
#include "settings_storage.h"
#include "led.h"
#include "adxl371.h"
#include "icm45686.h"
#include "bmp581.h"
#include "mmc5983ma.h"


#if FIRM_HARDWARE_VERSION == VERSION_V1_0

#define FLASH_MAIN_SETTINGS_SECTOR 0
#define FLASH_MOCK_SETTINGS_SECTOR 1

static uint32_t get_sector(const StoragePartition_t partition) {
  if (partition == PARTITION_SETTINGS_MOCK)
    return FLASH_MOCK_SETTINGS_SECTOR;
  return FLASH_MAIN_SETTINGS_SECTOR;
}

static int settings_storage_read(const StoragePartition_t partition, uint8_t *buffer, size_t len) {
  w25q128jv_read_sector(buffer, get_sector(partition), 0, len);
  return 0;
}

static int settings_storage_write(const StoragePartition_t partition, uint8_t *buffer, size_t len) {
  w25q128jv_erase_sector(get_sector(partition));
  w25q128jv_write_sector(buffer, get_sector(partition), 0, len);
  return 0;
}

static uint64_t settings_read_uid(void) {
  uint64_t uid;
  w25q128jv_read_UID((uint8_t *)&uid, 8);
  return uid;
}

int firm_init_hardware(void) {
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

  return 0;
}

#endif