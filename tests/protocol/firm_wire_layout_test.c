#include <stddef.h>

#include "identifiers.h"
#include "log_info.h"
#include "packets.h"
#include "utest.h"

UTEST(wire_layout, data_packet) {
  /* DataPacket_t is one double timestamp plus 20 floats. */
  ASSERT_EQ(8u + 20u * 4u, sizeof(DataPacket_t));
  ASSERT_EQ(0u, offsetof(DataPacket_t, timestamp_seconds));
}

UTEST(wire_layout, identifiers) {
  ASSERT_EQ(0x01, ID_DATA_PACKET);
  ASSERT_EQ(0x02, ID_GET_DEVICE_INFO);
  ASSERT_EQ(0x03, ID_GET_DEVICE_CONFIG);
  ASSERT_EQ(0x04, ID_SET_DEVICE_CONFIG);
  ASSERT_EQ(0x05, ID_REBOOT);
  ASSERT_EQ(0x06, ID_MOCK_REQUEST);
  ASSERT_EQ(0x07, ID_SET_MAG_CALIBRATON);
  ASSERT_EQ(0x08, ID_SET_IMU_CALIBRATON);
  ASSERT_EQ(0x09, ID_GET_CALIBRATION);
  ASSERT_EQ(0x0A, ID_CANCEL_REQUEST);
  ASSERT_EQ('B', ID_BAROMETER);
  ASSERT_EQ('I', ID_IMU);
  ASSERT_EQ('M', ID_MAGNETOMETER);
  ASSERT_EQ('A', ID_HIGH_G_ACCELEROMETER);
  ASSERT_EQ('H', ID_MOCK_HEADER);
}

UTEST(wire_layout, log_header_text) { ASSERT_STREQ("FIRM LOG v1.4\n", FIRM_LOG_HEADER_TEXT); }

UTEST_MAIN()
