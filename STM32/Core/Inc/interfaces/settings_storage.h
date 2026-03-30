#pragma once
#include "system_settings.h"
#include <stdint.h>
#include <stddef.h>

typedef enum {
  PARTITION_SETTINGS_MAIN,
  PARTITION_SETTINGS_MOCK,
} StoragePartition_t;

typedef struct {
  void(*read_settings)(const StoragePartition_t partition, uint8_t* buf, size_t len);
  void(*write_settings)(const StoragePartition_t partition, uint8_t* buf, size_t len);
  uint64_t(*read_uid)(void);
} SettingsStorageInterface_t;

void settings_storage_init(SettingsStorageInterface_t *interface);

void settings_storage_set_partition(StoragePartition_t partition);

void settings_write_to_storage(SystemSettings_t *settings);

void settings_read_from_storage(SystemSettings_t *settings);

uint64_t settings_read_storage_uid(void);
