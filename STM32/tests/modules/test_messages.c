#include "messages.h"
#include "utest.h"

static bool dispatched_command;

void dispatch_command(const uint8_t *command_bytes) {
  (void)command_bytes;
  dispatched_command = true;
}

uint32_t dispatch_mock_msg(const uint8_t *mock_message) {
  (void)mock_message;
  return 1U;
}

struct messages {
  size_t case_index;
};

UTEST_I_SETUP(messages) {
  dispatched_command = false;
  utest_fixture->case_index = utest_index;
}
UTEST_I_TEARDOWN(messages) {}

static const struct {
  uint8_t id;
  int expected_size;
} parse_msg_cases[] = {
    {0, -1},
    {1, -1},
    {2, 0},
    {3, 0},
    {4, sizeof(DeviceConfig_t)},
    {5, 0},
    {6, 0},
    {7, sizeof(Calibration_t)},
    {8, sizeof(Calibration_t) * 2},
    {9, 0},
    {10, 0},
    {66, sizeof(uint32_t) + sizeof(BMP581RawData_t)},
    {73, sizeof(uint32_t) + sizeof(ICM45686RawData_t)},
    {77, sizeof(uint32_t) + sizeof(MMC5983MARawData_t)},
    {65, sizeof(uint32_t) + sizeof(ADXL371RawData_t)},
    {11, -1},
    {86, -1},
};

UTEST_I(messages, parse_msg_no_size, 17) {
  ASSERT_EQ(parse_msg_cases[utest_fixture->case_index].expected_size,
            parse_message_id(parse_msg_cases[utest_fixture->case_index].id));
}

// Command IDs 2 through 10.
UTEST_I(messages, dispatch_valid_cmd, 9) {
  uint8_t id = (uint8_t)(2U + utest_fixture->case_index);
  uint8_t msg[5] = {id, 1, 7, 4, 0};
  ASSERT_EQ(0U, dispatch_message(msg));
  ASSERT_TRUE(dispatched_command);
}

static const uint8_t invalid_cmd_cases[] = {0, 66, 86, 11};

UTEST_I(messages, dispatch_invalid_cmd, 4) {
  uint8_t msg[5] = {invalid_cmd_cases[utest_fixture->case_index], 1, 7, 4, 0};
  dispatch_message(msg);
  ASSERT_FALSE(dispatched_command);
}

static const uint8_t mock_cases[] = {66, 73, 77, 65};

UTEST_I(messages, dispatch_mock, 4) {
  uint8_t msg[5] = {mock_cases[utest_fixture->case_index], 1, 7, 4, 0};
  ASSERT_EQ(1U, dispatch_message(msg));
  ASSERT_FALSE(dispatched_command);
}

UTEST_MAIN()
