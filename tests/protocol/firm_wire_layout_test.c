#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "identifiers.h"
#include "log_info.h"
#include "packets.h"

static int expect_size(const char *name, size_t actual, size_t expected) {
  if (actual == expected) {
    return 0;
  }
  fprintf(stderr, "%s is %zu bytes, expected %zu\n", name, actual, expected);
  return 1;
}

static int expect_id(const char *name, int actual, int expected) {
  if (actual == expected) {
    return 0;
  }
  fprintf(stderr, "%s is 0x%02x, expected 0x%02x\n", name, actual, expected);
  return 1;
}

int main(void) {
  int failed = 0;

  /* DataPacket_t is one double timestamp plus 20 floats. */
  failed |= expect_size("DataPacket_t", sizeof(DataPacket_t), 8u + 20u * 4u);
  failed |= expect_size("timestamp_seconds",
                        offsetof(DataPacket_t, timestamp_seconds), 0u);

  failed |= expect_id("ID_DATA_PACKET", ID_DATA_PACKET, 0x01);
  failed |= expect_id("ID_GET_DEVICE_INFO", ID_GET_DEVICE_INFO, 0x02);
  failed |= expect_id("ID_GET_DEVICE_CONFIG", ID_GET_DEVICE_CONFIG, 0x03);
  failed |= expect_id("ID_SET_DEVICE_CONFIG", ID_SET_DEVICE_CONFIG, 0x04);
  failed |= expect_id("ID_REBOOT", ID_REBOOT, 0x05);
  failed |= expect_id("ID_MOCK_REQUEST", ID_MOCK_REQUEST, 0x06);
  failed |= expect_id("ID_SET_MAG_CALIBRATON", ID_SET_MAG_CALIBRATON, 0x07);
  failed |= expect_id("ID_SET_IMU_CALIBRATON", ID_SET_IMU_CALIBRATON, 0x08);
  failed |= expect_id("ID_GET_CALIBRATION", ID_GET_CALIBRATION, 0x09);
  failed |= expect_id("ID_CANCEL_REQUEST", ID_CANCEL_REQUEST, 0x0A);
  failed |= expect_id("ID_BAROMETER", ID_BAROMETER, 'B');
  failed |= expect_id("ID_IMU", ID_IMU, 'I');
  failed |= expect_id("ID_MAGNETOMETER", ID_MAGNETOMETER, 'M');
  failed |= expect_id("ID_HIGH_G_ACCELEROMETER", ID_HIGH_G_ACCELEROMETER, 'A');
  failed |= expect_id("ID_MOCK_HEADER", ID_MOCK_HEADER, 'H');

  if (strcmp(FIRM_LOG_HEADER_TEXT, "FIRM LOG v1.4\n") != 0) {
    fprintf(stderr, "FIRM_LOG_HEADER_TEXT is %s\n", FIRM_LOG_HEADER_TEXT);
    failed = 1;
  }

  return failed;
}
