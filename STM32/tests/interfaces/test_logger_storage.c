#include <stdio.h>
#include <string.h>

#include "fake_logger.h"
#include "logger_storage.h"
#include "utest.h"

#define TEST_SECTOR_SIZE 16U
#define TEST_LOG_FILENAME "logger_storage_test.frm"

static uint8_t buffer_a[TEST_SECTOR_SIZE];
static uint8_t buffer_b[TEST_SECTOR_SIZE];

struct logger_storage {
  int unused;
};

UTEST_F_SETUP(logger_storage) {
  ASSERT_EQ(0, fake_logger_init());

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

  ASSERT_EQ(0, logger_storage_init(&logger_interface));
  ASSERT_EQ(0, logger_create_file(TEST_LOG_FILENAME, 1024U));
}

UTEST_F_TEARDOWN(logger_storage) { fake_logger_cleanup_logs(); }

UTEST_F(logger_storage, allocations_use_active_buffer_until_overflow) {
  uint8_t *first = logger_storage_malloc_capacity(6U);
  uint8_t *second = logger_storage_malloc_capacity(4U);

  ASSERT_NE(NULL, first);
  ASSERT_NE(NULL, second);
  ASSERT_EQ((void *)&buffer_a[0], first);
  ASSERT_EQ((void *)&buffer_a[6], second);

  memset(first, 0x11, 6U);
  memset(second, 0x22, 4U);
  ASSERT_EQ(0x11, buffer_a[0]);
  ASSERT_EQ(0x22, buffer_a[6]);
}

UTEST_F(logger_storage, when_active_buffer_would_overflow_it_swaps_to_standby_buffer) {
  uint8_t *first = logger_storage_malloc_capacity(12U);
  uint8_t *second = logger_storage_malloc_capacity(5U);

  ASSERT_NE(NULL, first);
  ASSERT_NE(NULL, second);

  // 12 + 5 > 16, so second allocation should flush active and start at standby[0].
  ASSERT_EQ((void *)&buffer_a[0], first);
  ASSERT_EQ((void *)&buffer_b[0], second);

  memset(first, 0x33, 12U);
  memset(second, 0x44, 5U);
  ASSERT_EQ(0x33, buffer_a[0]);
  ASSERT_EQ(0x44, buffer_b[0]);

  // ensure that buffer A has the last 4 bytes padded with zeroes
  uint32_t padding;
  memcpy(&padding, &buffer_a[12], 4);
  ASSERT_EQ(0x00000000U, padding);
}

UTEST_F(logger_storage, double_buffering_swaps_back_after_standby_overflow) {
  uint8_t *first = logger_storage_malloc_capacity(12U);
  uint8_t *second = logger_storage_malloc_capacity(8U);
  uint8_t *third = logger_storage_malloc_capacity(9U);

  ASSERT_NE(NULL, first);
  ASSERT_NE(NULL, second);
  ASSERT_NE(NULL, third);

  // first on active, second on standby after first overflow, third back on active after standby
  // overflow.
  ASSERT_EQ((void *)&buffer_a[0], first);
  ASSERT_EQ((void *)&buffer_b[0], second);
  ASSERT_EQ((void *)&buffer_a[0], third);

  memset(first, 0x51, 12U);
  memset(second, 0x62, 8U);
  memset(third, 0x73, 9U);
  ASSERT_EQ(0x73, buffer_a[0]);
  ASSERT_EQ(0x62, buffer_b[0]);
}

UTEST_F(logger_storage, overflow_flushes_zero_padded_tail_to_file) {
  uint8_t *first = logger_storage_malloc_capacity(12U);

  ASSERT_NE(NULL, first);

  memset(first, 0x33, 12U);

  uint8_t *second = logger_storage_malloc_capacity(5U);
  ASSERT_NE(NULL, second);

  memset(second, 0x44, 5U);

  FILE *log_file = fopen(FAKE_LOG_DIR "/" TEST_LOG_FILENAME, "rb");
  ASSERT_NE(NULL, log_file);

  uint8_t sector[TEST_SECTOR_SIZE] = {0};
  ASSERT_EQ(TEST_SECTOR_SIZE, (uint16_t)fread(sector, 1U, sizeof(sector), log_file));
  fclose(log_file);

  ASSERT_EQ(0x33, sector[0]);
  ASSERT_EQ(0x33, sector[11]);
  ASSERT_EQ(0x00, sector[12]);
  ASSERT_EQ(0x00, sector[13]);
  ASSERT_EQ(0x00, sector[14]);
  ASSERT_EQ(0x00, sector[15]);
}

UTEST_MAIN()
