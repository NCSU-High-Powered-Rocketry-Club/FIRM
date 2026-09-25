#include <unity.h>

#include "messages.h"

static bool dispatched_command;

void setUp() {
  dispatched_command = false;
}
void tearDown() {}

void dispatch_command(const uint8_t *command_bytes) {
	(void)command_bytes;
  dispatched_command = true;
}

uint32_t dispatch_mock_msg(const uint8_t *mock_message) {
	(void)mock_message;
	return 1U;
}

TEST_CASE(0, -1)
TEST_CASE(1, -1)
TEST_CASE(2, 0)
TEST_CASE(3, 0)
TEST_CASE(4, sizeof(DeviceConfig_t))
TEST_CASE(5, 0)
TEST_CASE(6, 0)
TEST_CASE(7, sizeof(Calibration_t))
TEST_CASE(8, sizeof(Calibration_t) * 2)
TEST_CASE(9, 0)
TEST_CASE(10, 0)
TEST_CASE(66, sizeof(uint32_t) + sizeof(BMP581RawData_t))
TEST_CASE(73, sizeof(uint32_t) + sizeof(ICM45686RawData_t))
TEST_CASE(77, sizeof(uint32_t) + sizeof(MMC5983MARawData_t))
TEST_CASE(65, sizeof(uint32_t) + sizeof(ADXL371RawData_t))
TEST_CASE(11, -1)
TEST_CASE(86, -1)
void test_parse_msg_no_size(uint32_t num, uint32_t exp) {
  uint8_t msg = (uint8_t)num;
  TEST_ASSERT_EQUAL_INT((int)exp, parse_message_id(msg));
}

TEST_RANGE([2, 10, 1])
void test_dispatch_valid_cmd(uint32_t num) {
  uint8_t id = (uint8_t)num;
  uint8_t msg[5] = {id, 1, 7, 4, 0};
  TEST_ASSERT_EQUAL_INT(0, dispatch_message(msg));
  TEST_ASSERT_TRUE(dispatched_command);
}

TEST_CASE(0)
TEST_CASE(66)
TEST_CASE(86)
TEST_CASE(11)
void test_dispatch_invalid_cmd(uint32_t num) {
  uint8_t id = (uint8_t)num;
  uint8_t msg[5] = {id, 1, 7, 4, 0};
  dispatch_message(msg);
  TEST_ASSERT_FALSE(dispatched_command);
}

TEST_CASE(66)
TEST_CASE(73)
TEST_CASE(77)
TEST_CASE(65)
void test_dispatch_mock(uint32_t num) {
  uint8_t id = (uint8_t)num;
  uint8_t msg[5] = {id, 1, 7, 4, 0};
  TEST_ASSERT_EQUAL(1, dispatch_message(msg));
  TEST_ASSERT_FALSE(dispatched_command);
  
}