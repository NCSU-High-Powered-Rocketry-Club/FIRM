#include <unity.h>

#include <string.h>

#include "commands.h"
#include "mocking_handler.h"

static SystemSettings_t stub_settings;
static TransmitFrame_t captured_frame;
static bool response_was_queued;

const SystemSettings_t *get_settings(void) { return &stub_settings; }

int settings_write_calibration(Calibration_t *accel_calibration, Calibration_t *gyro_calibration,
                               Calibration_t *mag_calibration, Calibration_t *high_g_calibration) {
  (void)accel_calibration;
  (void)gyro_calibration;
  (void)mag_calibration;
  (void)high_g_calibration;
  return 0;
}

int settings_write_firm_settings(SystemSettings_t *firm_settings) {
  (void)firm_settings;
  return 0;
}

bool mocking_handler_start_mock(void) { return false; }

bool mocking_handler_cancel_mock(void) { return false; }

static void capture_response(TransmitFrame_t *frame) {
  captured_frame = *frame;
  response_was_queued = true;
}

void setUp(void) {
  memset(&stub_settings, 0, sizeof(stub_settings));
  memset(&captured_frame, 0, sizeof(captured_frame));
  response_was_queued = false;
  commands_set_response_queue(capture_response);
}

void tearDown(void) {}

void test_get_device_info_response_length_includes_identifier_byte(void) {
  stub_settings.device_uid = 0x1122334455667788ULL;
  memcpy(stub_settings.firmware_version, "v2.2.0", 7U);
  const uint8_t command[] = {(uint8_t)ID_GET_DEVICE_INFO};

  dispatch_command(command);

  TEST_ASSERT_TRUE(response_was_queued);
  TEST_ASSERT_EQUAL_UINT8(ID_GET_DEVICE_INFO, captured_frame.payload[0]);
  TEST_ASSERT_EQUAL_UINT16(1U + sizeof(DeviceInfo_t), captured_frame.payload_len);
  TEST_ASSERT_EQUAL_MEMORY(&stub_settings.device_uid, &captured_frame.payload[1], sizeof(uint64_t));
  TEST_ASSERT_EQUAL_MEMORY(stub_settings.firmware_version,
                           &captured_frame.payload[1U + sizeof(uint64_t)],
                           FIRM_FIRMWARE_VERSION_LEN);
}
