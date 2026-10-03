#include <string.h>

#include "commands.h"
#include "mocking_handler.h"
#include "utest.h"

// Link-seam stubs for the settings_manager and mocking_handler calls in commands.c.
static const SystemSettings_t *stub_settings;
static int get_settings_calls;

const SystemSettings_t *get_settings(void) {
  get_settings_calls++;
  return stub_settings;
}
int settings_write_firm_settings(SystemSettings_t *settings) {
  (void)settings;
  return 0;
}
int settings_write_calibration(Calibration_t *accel, Calibration_t *gyro, Calibration_t *mag,
                               Calibration_t *high_g) {
  (void)accel;
  (void)gyro;
  (void)mag;
  (void)high_g;
  return 0;
}
bool mocking_handler_start_mock(void) { return false; }
bool mocking_handler_cancel_mock(void) { return false; }

static TransmitFrame_t captured_frame;
static bool response_was_queued;

static void capture_response(TransmitFrame_t *frame) {
  captured_frame = *frame;
  response_was_queued = true;
}

struct commands {
  int unused;
};

UTEST_F_SETUP(commands) {
  memset(&captured_frame, 0, sizeof(captured_frame));
  response_was_queued = false;
  stub_settings = NULL;
  get_settings_calls = 0;
  commands_set_response_queue(capture_response);
}

UTEST_F_TEARDOWN(commands) {}

UTEST_F(commands, get_device_info_response_length_includes_identifier_byte) {
  SystemSettings_t settings = {0};
  settings.device_uid = 0x1122334455667788ULL;
  memcpy(settings.firmware_version, "v2.2.0", 7U);
  const uint8_t command[] = {(uint8_t)ID_GET_DEVICE_INFO};

  stub_settings = &settings;
  dispatch_command(command);

  ASSERT_EQ(1, get_settings_calls);
  ASSERT_TRUE(response_was_queued);
  ASSERT_EQ((uint8_t)ID_GET_DEVICE_INFO, captured_frame.payload[0]);
  ASSERT_EQ(1U + sizeof(DeviceInfo_t), (size_t)captured_frame.payload_len);
  ASSERT_MEMEQ(&settings.device_uid, &captured_frame.payload[1], sizeof(uint64_t));
  ASSERT_MEMEQ(settings.firmware_version, &captured_frame.payload[1U + sizeof(uint64_t)],
               FIRM_FIRMWARE_VERSION_LEN);
}

UTEST_MAIN()
