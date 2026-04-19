#include <unity.h>
#include "fake_logger.h"
#include "logger_storage.h"
#include "logger.h"

TEST_SOURCE_FILE("logger_storage.c");

static uint8_t buf1[8192];
static uint8_t buf2[8192];

void setUp(void) {
  TEST_ASSERT_EQUAL_INT(0, fake_logger_init());

  LoggerStorageInterface_t logger_interface = {
    .file_exists = fake_file_exists,
    .create_file = fake_create_file,
    .is_write_ready = fake_is_write_ready,
    .write_sector = fake_write_sector,
    .active_buffer = buf1,
    .standby_buffer = buf2,
    .buffer_size = 8192,
  };

  TEST_ASSERT_EQUAL_INT(0, logger_storage_init(&logger_interface));
}

void tearDown(void) {
  fake_logger_cleanup_logs();
}

void test_create_log_files(void) {
  TEST_ASSERT_EQUAL_INT(0, create_log());
  TEST_ASSERT_TRUE(fake_file_exists("log1.frm"));

  TEST_ASSERT_EQUAL_INT(0, create_log());
  TEST_ASSERT_TRUE(fake_file_exists("log2.frm"));
}