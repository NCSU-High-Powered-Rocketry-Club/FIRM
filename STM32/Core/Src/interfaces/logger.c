#include "logger.h"

#include <string.h>

static size_t barometer_data_size = 0;
static size_t imu_data_size = 0;
static size_t magnetometer_data_size = 0;
static size_t high_g_data_size = 0;

int create_log(void) {
  char file_name[32] = {'\0'};
  int file_index = 0;

  // Find the next available log file
  do {
    file_index++;
    sprintf(file_name, "log%i.frm", file_index);
  } while (file_exists(file_name));

  // create and open the new file, with a set amount of allocated space
  return logger_create_file(file_name, 200000000ULL);
}

int logger_write_header(SystemSettings_t header_info) {
  const char *firm_log_header = FIRM_LOG_HEADER_TEXT;
  size_t header_len = strlen(firm_log_header);
  size_t system_settings_len = sizeof(SystemSettings_t);

  char *header_storage = (char *)logger_storage_malloc_capacity(header_len + system_settings_len);
  if (header_storage == NULL)
    return 1;

  // copy "FIRM LOG" text
  // NOLINTNEXTLINE(bugprone-not-null-terminated-result)
  memcpy(header_storage, firm_log_header, header_len);
  header_storage += header_len;

  // copy in system settings
  memcpy(header_storage, &header_info, system_settings_len);

  return 0;
}

void *logger_malloc_raw_storage(Sensors_t sensor_id, uint32_t timestamp) {
  // the required log file space is the id byte (sensor_id), the timestamp, and the data itself.
  size_t required_size = sizeof(timestamp) + sizeof(char);
  switch (sensor_id) {
  case BAROMETER:
    required_size += barometer_data_size;
    break;
  case IMU:
    required_size += imu_data_size;
    break;
  case MAGNETOMETER:
    required_size += magnetometer_data_size;
    break;
  case HIGH_G_ACCELEROMETER:
    required_size += high_g_data_size;
    break;
  default:
    return NULL;
  }

  // Reserve storage, write metadata, then return pointer location at start of data
  uint8_t *entry = logger_storage_malloc_capacity(required_size);
  if (entry == NULL)
    return NULL;

  entry[0] = (uint8_t)sensor_id;
  memcpy(entry + sizeof(char), &timestamp, sizeof(timestamp));
  return entry + sizeof(char) + sizeof(timestamp);
}

void logger_set_sensor_info(size_t barometer_size, size_t imu_size, size_t magnetometer_size,
                            size_t high_g_size) {
  barometer_data_size = barometer_size;
  imu_data_size = imu_size;
  magnetometer_data_size = magnetometer_size;
  high_g_data_size = high_g_size;
}