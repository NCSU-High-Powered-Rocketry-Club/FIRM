#include "settings_storage.h"

static SettingsStorageInterface_t settings_interface;
static StoragePartition_t active_partition = PARTITION_SETTINGS_MAIN;

void settings_storage_init(SettingsStorageInterface_t *interface) {
  settings_interface = *interface;
}

void settings_storage_set_partition(StoragePartition_t partition) {
  active_partition = partition;
}

void settings_write_to_storage(SystemSettings_t *settings) {
  settings_interface.write_settings(active_partition, (uint8_t *)settings, sizeof(SystemSettings_t));
}

void settings_read_from_storage(SystemSettings_t *settings) {
  settings_interface.read_settings(active_partition, (uint8_t *)settings, sizeof(SystemSettings_t));
}

uint64_t settings_read_storage_uid(void) {
  return settings_interface.read_uid();
}