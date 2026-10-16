#include "messages.h"
#include "utest.h"

static bool dispatched_command;

void dispatch_command(const uint8_t *command_bytes) {
  (void)command_bytes;
  dispatched_command = true;
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
    {6, -1},
    {7, sizeof(Calibration_t)},
    {8, sizeof(Calibration_t) * 2},
    {9, 0},
    {10, 0},
    {66, -1},
    {73, -1},
    {77, -1},
    {65, -1},
    {11, -1},
    {86, -1},
};

UTEST_I(messages, parse_msg_no_size, 17) {
  ASSERT_EQ(parse_msg_cases[utest_fixture->case_index].expected_size,
            parse_message_id(parse_msg_cases[utest_fixture->case_index].id));
}

static const uint8_t valid_cmd_ids[] = {2, 3, 4, 5, 7, 8, 9, 10};

UTEST_I(messages, dispatch_valid_cmd, 8) {
  uint8_t id = valid_cmd_ids[utest_fixture->case_index];
  uint8_t msg[5] = {id, 1, 7, 4, 0};
  dispatch_message(msg);
  ASSERT_TRUE(dispatched_command);
}

// Includes the retired mock request (6) and the sensor log record IDs 'B', 'I', 'M', 'A'
// (66, 73, 77, 65), which must not be dispatched as commands.
static const uint8_t invalid_cmd_cases[] = {0, 6, 66, 73, 77, 65, 86, 11};

UTEST_I(messages, dispatch_invalid_cmd, 8) {
  uint8_t msg[5] = {invalid_cmd_cases[utest_fixture->case_index], 1, 7, 4, 0};
  dispatch_message(msg);
  ASSERT_FALSE(dispatched_command);
}

UTEST_MAIN()
