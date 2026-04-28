#include "messages.h"

size_t parse_message_id(uint8_t identifier_byte) {
  Identifiers_t id = (Identifiers_t)identifier_byte;
  switch (id) {
  case ID_GET_DEVICE_INFO:
  case ID_GET_DEVICE_CONFIG:
  case ID_REBOOT:
  case ID_MOCK_REQUEST:
  case ID_CANCEL_REQUEST:
  case ID_GET_CALIBRATION:
    return 0;
  case ID_SET_DEVICE_CONFIG:
    return 10; // placeholder
  case ID_SET_MAG_CALIBRATON:
    return sizeof(Calibration_t);
  case ID_SET_IMU_CALIBRATON:
    return sizeof(Calibration_t) * 2;
  case ID_BAROMETER:
    return sizeof(uint32_t) + sizeof(BMP581RawData_t);
  case ID_IMU:
    return sizeof(uint32_t) + sizeof(ICM45686RawData_t);
  case ID_MAGNETOMETER:
    return sizeof(uint32_t) + sizeof(MMC5983MARawData_t);
  case ID_HIGH_G_ACCELEROMETER:
    return sizeof(uint32_t) + sizeof(ADXL371RawData_t);
  case ID_DATA_PACKET:
  default:
    return -1;
  }
}

uint32_t dispatch_message(const uint8_t *message) {
  Identifiers_t id = (Identifiers_t)message[0];
  switch (id) {
  case ID_GET_DEVICE_INFO:
  case ID_GET_DEVICE_CONFIG:
  case ID_SET_DEVICE_CONFIG:
  case ID_REBOOT:
  case ID_MOCK_REQUEST:
  case ID_SET_MAG_CALIBRATON:
  case ID_SET_IMU_CALIBRATON:
  case ID_GET_CALIBRATION:
  case ID_CANCEL_REQUEST:
    dispatch_command(message);
    return 0;
  case ID_BAROMETER:
  case ID_IMU:
  case ID_MAGNETOMETER:
  case ID_HIGH_G_ACCELEROMETER:
  case ID_MOCK_HEADER:
    return dispatch_mock_msg(message);
  case ID_DATA_PACKET:
  default:
    return 0;
  }
}
