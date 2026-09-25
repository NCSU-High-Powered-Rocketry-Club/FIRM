#include <unity.h>

#include <stddef.h>

#include "messages.h"

static bool dispatched_command;

void setUp(void) { dispatched_command = false; }

void tearDown(void) {}

void dispatch_command(const uint8_t *command_bytes) {
  (void)command_bytes;
  dispatched_command = true;
}

uint32_t dispatch_mock_msg(const uint8_t *mock_message) {
  (void)mock_message;
  return 1U;
}

void test_parse_msg_no_size(void) {
  const struct {
    uint32_t num;
    int expected;
  } cases[] = {
      {0, -1},
      {1, -1},
      {2, 0},
      {3, 0},
      {4, (int)sizeof(DeviceConfig_t)},
      {5, 0},
      {6, 0},
      {7, (int)sizeof(Calibration_t)},
      {8, (int)sizeof(Calibration_t) * 2},
      {9, 0},
      {10, 0},
      {66, (int)(sizeof(uint32_t) + sizeof(BMP581RawData_t))},
      {73, (int)(sizeof(uint32_t) + sizeof(ICM45686RawData_t))},
      {77, (int)(sizeof(uint32_t) + sizeof(MMC5983MARawData_t))},
      {65, (int)(sizeof(uint32_t) + sizeof(ADXL371RawData_t))},
      {11, -1},
      {86, -1},
  };

  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    uint8_t msg = (uint8_t)cases[i].num;
    TEST_ASSERT_EQUAL_INT(cases[i].expected, parse_message_id(msg));
  }
}

void test_dispatch_valid_cmd(void) {
  for (uint32_t num = 2; num <= 10; num++) {
    dispatched_command = false;
    uint8_t id = (uint8_t)num;
    uint8_t msg[5] = {id, 1, 7, 4, 0};
    TEST_ASSERT_EQUAL_INT(0, dispatch_message(msg));
    TEST_ASSERT_TRUE(dispatched_command);
  }
}

void test_dispatch_invalid_cmd(void) {
  const uint32_t cases[] = {0, 66, 86, 11};
  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    dispatched_command = false;
    uint8_t id = (uint8_t)cases[i];
    uint8_t msg[5] = {id, 1, 7, 4, 0};
    dispatch_message(msg);
    TEST_ASSERT_FALSE(dispatched_command);
  }
}

void test_dispatch_mock(void) {
  const uint32_t cases[] = {66, 73, 77, 65};
  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    dispatched_command = false;
    uint8_t id = (uint8_t)cases[i];
    uint8_t msg[5] = {id, 1, 7, 4, 0};
    TEST_ASSERT_EQUAL(1, dispatch_message(msg));
    TEST_ASSERT_FALSE(dispatched_command);
  }
}
