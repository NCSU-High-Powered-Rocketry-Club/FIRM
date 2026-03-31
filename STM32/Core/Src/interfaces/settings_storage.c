#include "settings_storage.h"

static SettingsStorageInterface_t settings_interface;
static StoragePartition_t active_partition = PARTITION_SETTINGS_MAIN;

int settings_storage_init(SettingsStorageInterface_t *interface) {
  if (interface == NULL)
    return 1;
  settings_interface = *interface;
  return 0;
}

int settings_storage_set_partition(StoragePartition_t partition) {
  active_partition = partition;
  return 0;
}

int settings_write_to_storage(SystemSettings_t *settings) {
  if (settings == NULL || settings_interface.write_settings == NULL)
    return 1;
  settings_interface.write_settings(active_partition, (uint8_t *)settings, sizeof(SystemSettings_t));
  return 0;
}

int settings_read_from_storage(SystemSettings_t *settings) {
  if (settings == NULL || settings_interface.read_settings == NULL)
    return 1;
  settings_interface.read_settings(active_partition, (uint8_t *)settings, sizeof(SystemSettings_t));
  return 0;
}

uint64_t settings_read_storage_uid(void) {
  if (settings_interface.read_uid == NULL)
    return 1;
  return settings_interface.read_uid();
  return 0;
}