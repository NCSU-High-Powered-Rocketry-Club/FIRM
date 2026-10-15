#include "fake_logger.h"
#include "logger.h"
#include "logger_storage.h"
#include "utest.h"
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
#define ASSERT_HEADER_AT_OFFSET(offset, expected)                                                  \
  do {                                                                                             \
    ASSERT_MEMEQ(FIRM_LOG_HEADER_TEXT, &buf1[(offset)], HEADER_TEXT_LEN);                          \
    ASSERT_MEMEQ((expected), &buf1[(offset) + HEADER_TEXT_LEN], sizeof(SystemSettings_t));         \
  } while (0)

// Validates a raw entry format: sensor id, timestamp, then payload bytes.
#define ASSERT_RAW_ENTRY_LAYOUT(offset, sensor_id, timestamp, expected_payload, payload_len)       \
  do {                                                                                             \
    const uint32_t expected_timestamp = (timestamp);                                               \
    ASSERT_EQ((uint8_t)(sensor_id), buf1[(offset)]);                                               \
    ASSERT_MEMEQ(&expected_timestamp, &buf1[(offset) + 1U], sizeof(expected_timestamp));           \
    ASSERT_MEMEQ((expected_payload), &buf1[(offset) + RAW_ENTRY_OVERHEAD_BYTES], (payload_len));   \
  } while (0)

struct logger {
  int unused;
};

UTEST_F_SETUP(logger) {
  // Start each test from clean buffers and a fresh fake storage backend.
  ASSERT_EQ(0, fake_logger_init());
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

  ASSERT_EQ(0, logger_storage_init(&logger_interface));
}

UTEST_F_TEARDOWN(logger) { fake_logger_cleanup_logs(); }

UTEST_F(logger, create_log_files) {
  // Ensure that repeated create_log calls allocate increasing file names.
  ASSERT_EQ(0, create_log());
  ASSERT_TRUE(fake_file_exists("log1.frm"));

  ASSERT_EQ(0, create_log());
  ASSERT_TRUE(fake_file_exists("log2.frm"));

  ASSERT_EQ(0, create_log());
  ASSERT_TRUE(fake_file_exists("log3.frm"));
}

UTEST_F(logger, logger_write_header_writes_header_text_and_system_settings) {
  SystemSettings_t settings = make_settings(0x1122334455667788ULL, 'A', 250);

  ASSERT_EQ(0, create_log());
  ASSERT_EQ(0, logger_write_header(settings));

  ASSERT_HEADER_AT_OFFSET(0U, &settings);
}

UTEST_F(logger, logger_malloc_raw_storage_barometer_layout) {
  const uint32_t timestamp = 0x12345678U;
  const uint8_t payload[] = {0xA1U, 0xA2U, 0xA3U, 0xA4U};

  ASSERT_EQ(0, create_log());
  logger_set_sensor_info(sizeof(payload), 3U, 2U, 1U);

  uint8_t *data_ptr = logger_malloc_raw_storage(ID_BAROMETER, timestamp);
  ASSERT_NE(NULL, data_ptr);

  // Write payload through returned pointer and verify full entry layout in-place.
  memcpy(data_ptr, payload, sizeof(payload));
  ASSERT_RAW_ENTRY_LAYOUT(0U, ID_BAROMETER, timestamp, payload, sizeof(payload));
  ASSERT_EQ((void *)&buf1[RAW_ENTRY_OVERHEAD_BYTES], data_ptr);
}

UTEST_F(logger, logger_malloc_raw_storage_imu_layout) {
  const uint32_t timestamp = 0x0A0B0C0DU;
  const uint8_t payload[] = {0x10U, 0x20U, 0x30U};

  ASSERT_EQ(0, create_log());
  logger_set_sensor_info(1U, sizeof(payload), 2U, 4U);

  uint8_t *data_ptr = logger_malloc_raw_storage(ID_IMU, timestamp);
  ASSERT_NE(NULL, data_ptr);

  memcpy(data_ptr, payload, sizeof(payload));
  ASSERT_RAW_ENTRY_LAYOUT(0U, ID_IMU, timestamp, payload, sizeof(payload));
  ASSERT_EQ((void *)&buf1[RAW_ENTRY_OVERHEAD_BYTES], data_ptr);
}

UTEST_F(logger, logger_malloc_raw_storage_magnetometer_layout) {
  const uint32_t timestamp = 0xCAFEBABEU;
  const uint8_t payload[] = {0x55U, 0x44U, 0x33U, 0x22U, 0x11U};

  ASSERT_EQ(0, create_log());
  logger_set_sensor_info(2U, 1U, sizeof(payload), 3U);

  uint8_t *data_ptr = logger_malloc_raw_storage(ID_MAGNETOMETER, timestamp);
  ASSERT_NE(NULL, data_ptr);

  memcpy(data_ptr, payload, sizeof(payload));
  ASSERT_RAW_ENTRY_LAYOUT(0U, ID_MAGNETOMETER, timestamp, payload, sizeof(payload));
  ASSERT_EQ((void *)&buf1[RAW_ENTRY_OVERHEAD_BYTES], data_ptr);
}

UTEST_F(logger, logger_malloc_raw_storage_high_g_layout) {
  const uint32_t timestamp = 0x01020304U;
  const uint8_t payload[] = {0x77U, 0x88U};

  ASSERT_EQ(0, create_log());
  logger_set_sensor_info(3U, 2U, 1U, sizeof(payload));

  uint8_t *data_ptr = logger_malloc_raw_storage(ID_HIGH_G_ACCELEROMETER, timestamp);
  ASSERT_NE(NULL, data_ptr);

  memcpy(data_ptr, payload, sizeof(payload));
  ASSERT_RAW_ENTRY_LAYOUT(0U, ID_HIGH_G_ACCELEROMETER, timestamp, payload, sizeof(payload));
  ASSERT_EQ((void *)&buf1[RAW_ENTRY_OVERHEAD_BYTES], data_ptr);
}

UTEST_F(logger, sequential_raw_entries_append_without_overwriting_header) {
  const uint32_t ts_bar = 0x11111111U;
  const uint32_t ts_imu = 0x22222222U;
  const uint32_t ts_mag = 0x33333333U;
  const uint8_t bar_payload[] = {0xB1U, 0xB2U};
  const uint8_t imu_payload[] = {0xC1U, 0xC2U, 0xC3U};
  const uint8_t mag_payload[] = {0xD1U, 0xD2U, 0xD3U, 0xD4U};
  SystemSettings_t settings = make_settings(0x0102030405060708ULL, 'H', 100);
  uint8_t expected_header[HEADER_TOTAL_BYTES];

  ASSERT_EQ(0, create_log());
  ASSERT_EQ(0, logger_write_header(settings));
  // Keep a baseline copy to verify later appends do not clobber header bytes.
  memcpy(expected_header, buf1, sizeof(expected_header));

  logger_set_sensor_info(sizeof(bar_payload), sizeof(imu_payload), sizeof(mag_payload), 1U);

  size_t offset = HEADER_TOTAL_BYTES;
  uint8_t *bar_data = logger_malloc_raw_storage(ID_BAROMETER, ts_bar);
  ASSERT_NE(NULL, bar_data);
  memcpy(bar_data, bar_payload, sizeof(bar_payload));
  ASSERT_RAW_ENTRY_LAYOUT(offset, ID_BAROMETER, ts_bar, bar_payload, sizeof(bar_payload));
  ASSERT_EQ((void *)&buf1[offset + RAW_ENTRY_OVERHEAD_BYTES], bar_data);
  offset += RAW_ENTRY_OVERHEAD_BYTES + sizeof(bar_payload);

  uint8_t *imu_data = logger_malloc_raw_storage(ID_IMU, ts_imu);
  ASSERT_NE(NULL, imu_data);
  memcpy(imu_data, imu_payload, sizeof(imu_payload));
  ASSERT_RAW_ENTRY_LAYOUT(offset, ID_IMU, ts_imu, imu_payload, sizeof(imu_payload));
  ASSERT_EQ((void *)&buf1[offset + RAW_ENTRY_OVERHEAD_BYTES], imu_data);
  offset += RAW_ENTRY_OVERHEAD_BYTES + sizeof(imu_payload);

  uint8_t *mag_data = logger_malloc_raw_storage(ID_MAGNETOMETER, ts_mag);
  ASSERT_NE(NULL, mag_data);
  memcpy(mag_data, mag_payload, sizeof(mag_payload));
  ASSERT_RAW_ENTRY_LAYOUT(offset, ID_MAGNETOMETER, ts_mag, mag_payload, sizeof(mag_payload));
  ASSERT_EQ((void *)&buf1[offset + RAW_ENTRY_OVERHEAD_BYTES], mag_data);

  ASSERT_MEMEQ(expected_header, buf1, sizeof(expected_header));
}

UTEST_F(logger, second_header_and_new_raw_entries_append_after_existing_data) {
  const uint32_t ts_first = 0xABCDEF01U;
  const uint32_t ts_second = 0x10203040U;
  const uint32_t ts_after_second_header = 0x0BADBEEFU;
  const uint8_t first_payload[] = {0x01U, 0x02U, 0x03U, 0x04U};
  const uint8_t second_payload[] = {0x05U, 0x06U, 0x07U};
  const uint8_t post_header_payload[] = {0xAAU, 0xBBU};
  SystemSettings_t first_header = make_settings(0x1111222233334444ULL, 'F', 200);
  SystemSettings_t second_header = make_settings(0xAAAABBBBCCCCDDDDULL, 'S', 400);

  ASSERT_EQ(0, create_log());
  logger_set_sensor_info(sizeof(first_payload), sizeof(second_payload), sizeof(post_header_payload),
                         1U);

  ASSERT_EQ(0, logger_write_header(first_header));
  ASSERT_HEADER_AT_OFFSET(0U, &first_header);

  size_t offset = HEADER_TOTAL_BYTES;
  uint8_t *first_data = logger_malloc_raw_storage(ID_BAROMETER, ts_first);
  ASSERT_NE(NULL, first_data);
  memcpy(first_data, first_payload, sizeof(first_payload));
  ASSERT_RAW_ENTRY_LAYOUT(offset, ID_BAROMETER, ts_first, first_payload, sizeof(first_payload));
  ASSERT_EQ((void *)&buf1[offset + RAW_ENTRY_OVERHEAD_BYTES], first_data);
  offset += RAW_ENTRY_OVERHEAD_BYTES + sizeof(first_payload);

  uint8_t *second_data = logger_malloc_raw_storage(ID_IMU, ts_second);
  ASSERT_NE(NULL, second_data);
  memcpy(second_data, second_payload, sizeof(second_payload));
  ASSERT_RAW_ENTRY_LAYOUT(offset, ID_IMU, ts_second, second_payload, sizeof(second_payload));
  ASSERT_EQ((void *)&buf1[offset + RAW_ENTRY_OVERHEAD_BYTES], second_data);
  offset += RAW_ENTRY_OVERHEAD_BYTES + sizeof(second_payload);

  ASSERT_EQ(0, logger_write_header(second_header));
  // The second header should append at current offset, not overwrite prior data.
  ASSERT_HEADER_AT_OFFSET(offset, &second_header);
  offset += HEADER_TOTAL_BYTES;

  uint8_t *after_second_header = logger_malloc_raw_storage(ID_MAGNETOMETER, ts_after_second_header);
  ASSERT_NE(NULL, after_second_header);
  memcpy(after_second_header, post_header_payload, sizeof(post_header_payload));
  ASSERT_RAW_ENTRY_LAYOUT(offset, ID_MAGNETOMETER, ts_after_second_header, post_header_payload,
                          sizeof(post_header_payload));
  ASSERT_EQ((void *)&buf1[offset + RAW_ENTRY_OVERHEAD_BYTES], after_second_header);

  // Ensure the first header was preserved at the beginning.
  ASSERT_HEADER_AT_OFFSET(0U, &first_header);
}

UTEST_MAIN()
