#include <unity.h>

#include <string.h>

#include "fake_logger.h"
#include "logger_storage.h"

#define TEST_SECTOR_SIZE 16U

static uint8_t buffer_a[TEST_SECTOR_SIZE];
static uint8_t buffer_b[TEST_SECTOR_SIZE];

void setUp(void) {
  TEST_ASSERT_EQUAL_INT(0, fake_logger_init());

  // Fill with sentinels so tests can verify swap behavior without relying on zeroed bytes.
  memset(buffer_a, 0xA5, sizeof(buffer_a));
  memset(buffer_b, 0x5A, sizeof(buffer_b));

  LoggerStorageInterface_t logger_interface = {
      .file_exists = fake_file_exists,
      .create_file = fake_create_file,
      .is_write_ready = fake_is_write_ready,
      .write_sector = fake_write_sector,
      .active_buffer = buffer_a,
      .standby_buffer = buffer_b,
      .buffer_size = TEST_SECTOR_SIZE,
  };

  TEST_ASSERT_EQUAL_INT(0, logger_storage_init(&logger_interface));
  TEST_ASSERT_EQUAL_INT(0, logger_create_file("logger_storage_test.frm", 1024U));
}

void tearDown(void) { fake_logger_cleanup_logs(); }

void test_allocations_use_active_buffer_until_overflow(void) {
  uint8_t *first = logger_storage_malloc_capacity(6U);
  uint8_t *second = logger_storage_malloc_capacity(4U);

  TEST_ASSERT_NOT_NULL(first);
  TEST_ASSERT_NOT_NULL(second);
  TEST_ASSERT_EQUAL_PTR((void *)&buffer_a[0], first);
  TEST_ASSERT_EQUAL_PTR((void *)&buffer_a[6], second);

  memset(first, 0x11, 6U);
  memset(second, 0x22, 4U);
  TEST_ASSERT_EQUAL_HEX8(0x11, buffer_a[0]);
  TEST_ASSERT_EQUAL_HEX8(0x22, buffer_a[6]);
}

void test_when_active_buffer_would_overflow_it_swaps_to_standby_buffer(void) {
  uint8_t *first = logger_storage_malloc_capacity(12U);
  uint8_t *second = logger_storage_malloc_capacity(5U);

  TEST_ASSERT_NOT_NULL(first);
  TEST_ASSERT_NOT_NULL(second);

  // 12 + 5 > 16, so second allocation should flush active and start at standby[0].
  TEST_ASSERT_EQUAL_PTR((void *)&buffer_a[0], first);
  TEST_ASSERT_EQUAL_PTR((void *)&buffer_b[0], second);

  memset(first, 0x33, 12U);
  memset(second, 0x44, 5U);
  TEST_ASSERT_EQUAL_HEX8(0x33, buffer_a[0]);
  TEST_ASSERT_EQUAL_HEX8(0x44, buffer_b[0]);

  // ensure that buffer A has the last 4 bytes padded with zeroes
  uint32_t padding;
  memcpy(&padding, &buffer_a[12], 4);
  TEST_ASSERT_EQUAL_HEX32(0x00000000, padding);
}

void test_double_buffering_swaps_back_after_standby_overflow(void) {
  uint8_t *first = logger_storage_malloc_capacity(12U);
  uint8_t *second = logger_storage_malloc_capacity(8U);
  uint8_t *third = logger_storage_malloc_capacity(9U);

  TEST_ASSERT_NOT_NULL(first);
  TEST_ASSERT_NOT_NULL(second);
  TEST_ASSERT_NOT_NULL(third);

  // first on active, second on standby after first overflow, third back on active after standby
  // overflow.
  TEST_ASSERT_EQUAL_PTR((void *)&buffer_a[0], first);
  TEST_ASSERT_EQUAL_PTR((void *)&buffer_b[0], second);
  TEST_ASSERT_EQUAL_PTR((void *)&buffer_a[0], third);

  memset(first, 0x51, 12U);
  memset(second, 0x62, 8U);
  memset(third, 0x73, 9U);
  TEST_ASSERT_EQUAL_HEX8(0x73, buffer_a[0]);
  TEST_ASSERT_EQUAL_HEX8(0x62, buffer_b[0]);
}
