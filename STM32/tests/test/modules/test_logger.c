#include <unity.h>

#include "fake_logger.h"
#include "logger.h"
#include "logger_storage.h"
#include <string.h>

#define LOGGER_TEST_BUFFER_SIZE 8192U
#define RAW_ENTRY_OVERHEAD_BYTES 5U
#define HEADER_TEXT_LEN (sizeof(FIRM_LOG_HEADER_TEXT) - 1U)
#define HEADER_TOTAL_BYTES (HEADER_TEXT_LEN + sizeof(SystemSettings_t))

// Buffers used by logger_storage_init for active/standby write regions.
static uint8_t buf1[LOGGER_TEST_BUFFER_SIZE];
static uint8_t buf2[LOGGER_TEST_BUFFER_SIZE];

// Builds deterministic settings with non-zero calibration content for byte-wise asserts.
static SystemSettings_t make_settings(uint64_t uid, char device_char, uint16_t frequency_hz) {
  SystemSettings_t settings;
  memset(&settings, 0, sizeof(settings));

  settings.device_uid = uid;
  memset(settings.device_name, device_char, sizeof(settings.device_name));
  settings.usb_transfer_enabled = true;
  settings.uart_transfer_enabled = true;
  settings.i2c_transfer_enabled = false;
  settings.spi_transfer_enabled = true;
  memcpy(settings.firmware_version, "v1.4.42", sizeof(settings.firmware_version));
  settings.frequency_hz = frequency_hz;

  for (size_t i = 0; i < 3; i++) {
    settings.accel_cal.offset[i] = 0.1F * (float)(i + 1);
    settings.gyro_cal.offset[i] = 1.0F + 0.2F * (float)i;
    settings.mag_cal.offset[i] = 2.0F + 0.3F * (float)i;
    settings.high_g_cal.offset[i] = 3.0F + 0.4F * (float)i;
  }

  for (size_t i = 0; i < 9; i++) {
    settings.accel_cal.scale_matrix[i] = 1.0F + 0.01F * (float)i;
    settings.gyro_cal.scale_matrix[i] = 1.5F + 0.02F * (float)i;
    settings.mag_cal.scale_matrix[i] = 2.0F + 0.03F * (float)i;
    settings.high_g_cal.scale_matrix[i] = 2.5F + 0.04F * (float)i;
  }

  return settings;
}

// Validates serialized header format: text prefix followed by packed settings bytes.
static void assert_header_at_offset(size_t offset, const SystemSettings_t *expected) {
  TEST_ASSERT_EQUAL_MEMORY(FIRM_LOG_HEADER_TEXT, &buf1[offset], HEADER_TEXT_LEN);
  TEST_ASSERT_EQUAL_MEMORY(expected, &buf1[offset + HEADER_TEXT_LEN], sizeof(SystemSettings_t));
}

// Validates a raw entry format: sensor id, timestamp, then payload bytes.
static void assert_raw_entry_layout(size_t offset, Identifiers_t sensor_id, uint32_t timestamp,
                                    const uint8_t *expected_payload, size_t payload_len) {
  TEST_ASSERT_EQUAL_HEX8((uint8_t)sensor_id, buf1[offset]);
  TEST_ASSERT_EQUAL_MEMORY(&timestamp, &buf1[offset + 1U], sizeof(timestamp));

  if (payload_len > 0U) {
    TEST_ASSERT_EQUAL_MEMORY(expected_payload, &buf1[offset + RAW_ENTRY_OVERHEAD_BYTES],
                             payload_len);
  }
}

void setUp(void) {
  // Start each test from clean buffers and a fresh fake storage backend.
  TEST_ASSERT_EQUAL_INT(0, fake_logger_init());
  memset(buf1, 0, sizeof(buf1));
  memset(buf2, 0, sizeof(buf2));

  LoggerStorageInterface_t logger_interface = {
      .file_exists = fake_file_exists,
      .create_file = fake_create_file,
      .is_write_ready = fake_is_write_ready,
      .write_sector = fake_write_sector,
      .active_buffer = buf1,
      .standby_buffer = buf2,
      .buffer_size = LOGGER_TEST_BUFFER_SIZE,
  };

  TEST_ASSERT_EQUAL_INT(0, logger_storage_init(&logger_interface));
}

void tearDown(void) { fake_logger_cleanup_logs(); }

void test_create_log_files(void) {
  // Ensure that repeated create_log calls allocate increasing file names.
  TEST_ASSERT_EQUAL_INT(0, create_log());
  TEST_ASSERT_TRUE(fake_file_exists("log1.frm"));

  TEST_ASSERT_EQUAL_INT(0, create_log());
  TEST_ASSERT_TRUE(fake_file_exists("log2.frm"));

  TEST_ASSERT_EQUAL_INT(0, create_log());
  TEST_ASSERT_TRUE(fake_file_exists("log3.frm"));
}

void test_logger_write_header_writes_header_text_and_system_settings(void) {
  SystemSettings_t settings = make_settings(0x1122334455667788ULL, 'A', 250);

  TEST_ASSERT_EQUAL_INT(0, create_log());
  TEST_ASSERT_EQUAL_INT(0, logger_write_header(settings));

  assert_header_at_offset(0U, &settings);
}

void test_logger_malloc_raw_storage_barometer_layout(void) {
  const uint32_t timestamp = 0x12345678U;
  const uint8_t payload[] = {0xA1U, 0xA2U, 0xA3U, 0xA4U};

  TEST_ASSERT_EQUAL_INT(0, create_log());
  logger_set_sensor_info(sizeof(payload), 3U, 2U, 1U);

  uint8_t *data_ptr = logger_malloc_raw_storage(ID_BAROMETER, timestamp);
  TEST_ASSERT_NOT_NULL(data_ptr);

  // Write payload through returned pointer and verify full entry layout in-place.
  memcpy(data_ptr, payload, sizeof(payload));
  assert_raw_entry_layout(0U, ID_BAROMETER, timestamp, payload, sizeof(payload));
  TEST_ASSERT_EQUAL_PTR((void *)&buf1[RAW_ENTRY_OVERHEAD_BYTES], data_ptr);
}

void test_logger_malloc_raw_storage_imu_layout(void) {
  const uint32_t timestamp = 0x0A0B0C0DU;
  const uint8_t payload[] = {0x10U, 0x20U, 0x30U};

  TEST_ASSERT_EQUAL_INT(0, create_log());
  logger_set_sensor_info(1U, sizeof(payload), 2U, 4U);

  uint8_t *data_ptr = logger_malloc_raw_storage(ID_IMU, timestamp);
  TEST_ASSERT_NOT_NULL(data_ptr);

  memcpy(data_ptr, payload, sizeof(payload));
  assert_raw_entry_layout(0U, ID_IMU, timestamp, payload, sizeof(payload));
  TEST_ASSERT_EQUAL_PTR((void *)&buf1[RAW_ENTRY_OVERHEAD_BYTES], data_ptr);
}

void test_logger_malloc_raw_storage_magnetometer_layout(void) {
  const uint32_t timestamp = 0xCAFEBABEU;
  const uint8_t payload[] = {0x55U, 0x44U, 0x33U, 0x22U, 0x11U};

  TEST_ASSERT_EQUAL_INT(0, create_log());
  logger_set_sensor_info(2U, 1U, sizeof(payload), 3U);

  uint8_t *data_ptr = logger_malloc_raw_storage(ID_MAGNETOMETER, timestamp);
  TEST_ASSERT_NOT_NULL(data_ptr);

  memcpy(data_ptr, payload, sizeof(payload));
  assert_raw_entry_layout(0U, ID_MAGNETOMETER, timestamp, payload, sizeof(payload));
  TEST_ASSERT_EQUAL_PTR((void *)&buf1[RAW_ENTRY_OVERHEAD_BYTES], data_ptr);
}

void test_logger_malloc_raw_storage_high_g_layout(void) {
  const uint32_t timestamp = 0x01020304U;
  const uint8_t payload[] = {0x77U, 0x88U};

  TEST_ASSERT_EQUAL_INT(0, create_log());
  logger_set_sensor_info(3U, 2U, 1U, sizeof(payload));

  uint8_t *data_ptr = logger_malloc_raw_storage(ID_HIGH_G_ACCELEROMETER, timestamp);
  TEST_ASSERT_NOT_NULL(data_ptr);

  memcpy(data_ptr, payload, sizeof(payload));
  assert_raw_entry_layout(0U, ID_HIGH_G_ACCELEROMETER, timestamp, payload, sizeof(payload));
  TEST_ASSERT_EQUAL_PTR((void *)&buf1[RAW_ENTRY_OVERHEAD_BYTES], data_ptr);
}

void test_sequential_raw_entries_append_without_overwriting_header(void) {
  const uint32_t ts_bar = 0x11111111U;
  const uint32_t ts_imu = 0x22222222U;
  const uint32_t ts_mag = 0x33333333U;
  const uint8_t bar_payload[] = {0xB1U, 0xB2U};
  const uint8_t imu_payload[] = {0xC1U, 0xC2U, 0xC3U};
  const uint8_t mag_payload[] = {0xD1U, 0xD2U, 0xD3U, 0xD4U};
  SystemSettings_t settings = make_settings(0x0102030405060708ULL, 'H', 100);
  uint8_t expected_header[HEADER_TOTAL_BYTES];

  TEST_ASSERT_EQUAL_INT(0, create_log());
  TEST_ASSERT_EQUAL_INT(0, logger_write_header(settings));
  // Keep a baseline copy to verify later appends do not clobber header bytes.
  memcpy(expected_header, buf1, sizeof(expected_header));

  logger_set_sensor_info(sizeof(bar_payload), sizeof(imu_payload), sizeof(mag_payload), 1U);

  size_t offset = HEADER_TOTAL_BYTES;
  uint8_t *bar_data = logger_malloc_raw_storage(ID_BAROMETER, ts_bar);
  TEST_ASSERT_NOT_NULL(bar_data);
  memcpy(bar_data, bar_payload, sizeof(bar_payload));
  assert_raw_entry_layout(offset, ID_BAROMETER, ts_bar, bar_payload, sizeof(bar_payload));
  TEST_ASSERT_EQUAL_PTR((void *)&buf1[offset + RAW_ENTRY_OVERHEAD_BYTES], bar_data);
  offset += RAW_ENTRY_OVERHEAD_BYTES + sizeof(bar_payload);

  uint8_t *imu_data = logger_malloc_raw_storage(ID_IMU, ts_imu);
  TEST_ASSERT_NOT_NULL(imu_data);
  memcpy(imu_data, imu_payload, sizeof(imu_payload));
  assert_raw_entry_layout(offset, ID_IMU, ts_imu, imu_payload, sizeof(imu_payload));
  TEST_ASSERT_EQUAL_PTR((void *)&buf1[offset + RAW_ENTRY_OVERHEAD_BYTES], imu_data);
  offset += RAW_ENTRY_OVERHEAD_BYTES + sizeof(imu_payload);

  uint8_t *mag_data = logger_malloc_raw_storage(ID_MAGNETOMETER, ts_mag);
  TEST_ASSERT_NOT_NULL(mag_data);
  memcpy(mag_data, mag_payload, sizeof(mag_payload));
  assert_raw_entry_layout(offset, ID_MAGNETOMETER, ts_mag, mag_payload, sizeof(mag_payload));
  TEST_ASSERT_EQUAL_PTR((void *)&buf1[offset + RAW_ENTRY_OVERHEAD_BYTES], mag_data);

  TEST_ASSERT_EQUAL_MEMORY(expected_header, buf1, sizeof(expected_header));
}

void test_second_header_and_new_raw_entries_append_after_existing_data(void) {
  const uint32_t ts_first = 0xABCDEF01U;
  const uint32_t ts_second = 0x10203040U;
  const uint32_t ts_after_second_header = 0x0BADBEEFU;
  const uint8_t first_payload[] = {0x01U, 0x02U, 0x03U, 0x04U};
  const uint8_t second_payload[] = {0x05U, 0x06U, 0x07U};
  const uint8_t post_header_payload[] = {0xAAU, 0xBBU};
  SystemSettings_t first_header = make_settings(0x1111222233334444ULL, 'F', 200);
  SystemSettings_t second_header = make_settings(0xAAAABBBBCCCCDDDDULL, 'S', 400);

  TEST_ASSERT_EQUAL_INT(0, create_log());
  logger_set_sensor_info(sizeof(first_payload), sizeof(second_payload), sizeof(post_header_payload),
                         1U);

  TEST_ASSERT_EQUAL_INT(0, logger_write_header(first_header));
  assert_header_at_offset(0U, &first_header);

  size_t offset = HEADER_TOTAL_BYTES;
  uint8_t *first_data = logger_malloc_raw_storage(ID_BAROMETER, ts_first);
  TEST_ASSERT_NOT_NULL(first_data);
  memcpy(first_data, first_payload, sizeof(first_payload));
  assert_raw_entry_layout(offset, ID_BAROMETER, ts_first, first_payload, sizeof(first_payload));
  TEST_ASSERT_EQUAL_PTR((void *)&buf1[offset + RAW_ENTRY_OVERHEAD_BYTES], first_data);
  offset += RAW_ENTRY_OVERHEAD_BYTES + sizeof(first_payload);

  uint8_t *second_data = logger_malloc_raw_storage(ID_IMU, ts_second);
  TEST_ASSERT_NOT_NULL(second_data);
  memcpy(second_data, second_payload, sizeof(second_payload));
  assert_raw_entry_layout(offset, ID_IMU, ts_second, second_payload, sizeof(second_payload));
  TEST_ASSERT_EQUAL_PTR((void *)&buf1[offset + RAW_ENTRY_OVERHEAD_BYTES], second_data);
  offset += RAW_ENTRY_OVERHEAD_BYTES + sizeof(second_payload);

  TEST_ASSERT_EQUAL_INT(0, logger_write_header(second_header));
  // The second header should append at current offset, not overwrite prior data.
  assert_header_at_offset(offset, &second_header);
  offset += HEADER_TOTAL_BYTES;

  uint8_t *after_second_header = logger_malloc_raw_storage(ID_MAGNETOMETER, ts_after_second_header);
  TEST_ASSERT_NOT_NULL(after_second_header);
  memcpy(after_second_header, post_header_payload, sizeof(post_header_payload));
  assert_raw_entry_layout(offset, ID_MAGNETOMETER, ts_after_second_header, post_header_payload,
                          sizeof(post_header_payload));
  TEST_ASSERT_EQUAL_PTR((void *)&buf1[offset + RAW_ENTRY_OVERHEAD_BYTES], after_second_header);

  // Ensure the first header was preserved at the beginning.
  assert_header_at_offset(0U, &first_header);
}