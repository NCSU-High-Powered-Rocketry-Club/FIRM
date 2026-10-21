#include <string.h>

#include "commands.h"
#include "utest.h"

// Link-seam stubs for the settings_manager calls in commands.c.
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

static int calibration_changed_calls;

static void count_calibration_changed(void *ctx) {
  (void)ctx;
  calibration_changed_calls++;
}

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
  calibration_changed_calls = 0;
  commands_set_response_queue(capture_response);
  commands_register_calibration_changed(count_calibration_changed, NULL);
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

UTEST_F(commands, calibration_commands_reload_driver_calibration) {
  uint8_t mag_command[1U + sizeof(Calibration_t)] = {(uint8_t)ID_SET_MAG_CALIBRATON};
  uint8_t imu_command[1U + (2U * sizeof(Calibration_t))] = {(uint8_t)ID_SET_IMU_CALIBRATON};

  dispatch_command(mag_command);
  ASSERT_EQ(1, calibration_changed_calls);
  dispatch_command(imu_command);
  ASSERT_EQ(2, calibration_changed_calls);
}

struct command_payload {
  size_t case_index;
};

UTEST_I_SETUP(command_payload) { utest_fixture->case_index = utest_index; }
UTEST_I_TEARDOWN(command_payload) {}

static const struct {
  uint8_t id;
  int expected_size;
} payload_len_cases[] = {
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

UTEST_I(command_payload, len, 17) {
  ASSERT_EQ(payload_len_cases[utest_fixture->case_index].expected_size,
            command_payload_len(payload_len_cases[utest_fixture->case_index].id));
}

UTEST_MAIN()
