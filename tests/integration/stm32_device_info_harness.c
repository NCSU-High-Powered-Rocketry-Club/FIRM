// Real STM32 command dispatch with host-side dependency stubs.
//
// Reads length-prefixed commands from stdin and writes length-prefixed
// responses to stdout. Driven by test_get_device_info_pipeline.py, which
// expects the device UID and firmware version below.

#include "adxl371_packet.h"
#include "bmp581_packet.h"
#include "commands.h"
#include "icm45686_packet.h"
#include "mmc5983ma_packet.h"
#include "modules/transmit_frame.h"
#include "shared_data/system_settings.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const SystemSettings_t kSettings = {
    .device_uid = 0x1122334455667788ULL,
    .device_name = "INTEGRATION_TEST_DEVICE",
    .usb_transfer_enabled = true,
    .firmware_version = "v2.2.0",
    .frequency_hz = 100U,
};

static uint8_t response[256];
static uint32_t response_len;

const SystemSettings_t *get_settings(void) { return &kSettings; }
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
uint32_t dispatch_mock_msg(const uint8_t *message) {
  (void)message;
  return 0U;
}
uint32_t mocking_handler_time_from_ring(void) { return 0U; }
int mocking_handler_read_barometer(BMP581RawData_t *out) {
  (void)out;
  return 1;
}
int mocking_handler_read_imu(ICM45686RawData_t *out) {
  (void)out;
  return 1;
}
int mocking_handler_read_magnetometer(MMC5983MARawData_t *out) {
  (void)out;
  return 1;
}
int mocking_handler_read_high_g(ADXL371RawData_t *out) {
  (void)out;
  return 1;
}

static void capture_response(TransmitFrame_t *frame) {
  response_len = frame->payload_len;
  memcpy(response, frame->payload, response_len);
}

int main(void) {
  commands_set_response_queue(capture_response);
  for (;;) {
    uint32_t command_len = 0U;
    if (fread(&command_len, sizeof(command_len), 1U, stdin) != 1U)
      return 0;
    if (command_len == 0U || command_len > 255U)
      return 1;
    uint8_t command[255];
    if (fread(command, 1U, command_len, stdin) != command_len)
      return 1;

    response_len = 0U;
    dispatch_command(command);
    fwrite(&response_len, sizeof(response_len), 1U, stdout);
    if (response_len > 0U)
      fwrite(response, 1U, response_len, stdout);
    fflush(stdout);
  }
}
