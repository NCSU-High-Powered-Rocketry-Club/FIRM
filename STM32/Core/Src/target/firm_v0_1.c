#include "firm_v0_1.h"
#include "settings_storage.h"
#include "led.h"
#include "error_state_kalman_filter.h"
#include <string.h>


#if FIRM_HARDWARE_VERSION == FIRM_VERSION_V0_1

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
  SensorOrientations_t sensor_orientations = {0};
  memcpy(sensor_orientations.R_imu_to_board, eskf_v1_R_imu_to_board,
         sizeof(sensor_orientations.R_imu_to_board));
  memcpy(sensor_orientations.R_mag_to_board, eskf_v1_R_mag_to_board,
         sizeof(sensor_orientations.R_mag_to_board));
  if (eskf_set_sensor_orientations(&sensor_orientations)) {
    return 1;
  }

  // set up settings storage (w25q128jv NOR Flash)
  SettingsStorageInterface_t settings_storage_interface = {
    .read_settings = settings_storage_read,
    .write_settings = settings_storage_write,
    .read_uid = settings_read_uid,
  };
  w25q128jv_set_spi_settings(&hspi1, GPIOC, GPIO_PIN_4);
  if (w25q128jv_init()) {
    led_set_status(FLASH_CHIP_FAIL);
    return 1;
  }
  if (settings_storage_init(&settings_storage_interface)) {
    led_set_status(FLASH_CHIP_FAIL);
    return 1;
  }
  return 0;
}

#endif