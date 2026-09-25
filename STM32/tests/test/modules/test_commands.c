#include <unity.h>

#include <string.h>

#include "commands.h"
#include "mock_mocking_handler.h"
#include "mock_settings_manager.h"

static TransmitFrame_t captured_frame;
static bool response_was_queued;

static void capture_response(TransmitFrame_t *frame) {
  captured_frame = *frame;
  response_was_queued = true;
}

void setUp(void) {
  memset(&captured_frame, 0, sizeof(captured_frame));
  response_was_queued = false;
  commands_set_response_queue(capture_response);
}

void tearDown(void) {}

void test_get_device_info_response_length_includes_identifier_byte(void) {
  SystemSettings_t settings = {0};
  settings.device_uid = 0x1122334455667788ULL;
  memcpy(settings.firmware_version, "v2.2.0", 7U);
  const uint8_t command[] = {(uint8_t)ID_GET_DEVICE_INFO};

  get_settings_ExpectAndReturn(&settings);
  dispatch_command(command);

  TEST_ASSERT_TRUE(response_was_queued);
  TEST_ASSERT_EQUAL_UINT8(ID_GET_DEVICE_INFO, captured_frame.payload[0]);
  TEST_ASSERT_EQUAL_UINT16(1U + sizeof(DeviceInfo_t), captured_frame.payload_len);
  TEST_ASSERT_EQUAL_MEMORY(&settings.device_uid, &captured_frame.payload[1], sizeof(uint64_t));
  TEST_ASSERT_EQUAL_MEMORY(settings.firmware_version,
                           &captured_frame.payload[1U + sizeof(uint64_t)],
                           FIRM_FIRMWARE_VERSION_LEN);
}
