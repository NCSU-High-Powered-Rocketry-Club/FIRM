#include "messages.h"

int parse_message_id(uint8_t identifier_byte) {
  Identifiers_t id = (Identifiers_t)identifier_byte;
  switch (id) {
  case ID_GET_DEVICE_INFO:
  case ID_GET_DEVICE_CONFIG:
  case ID_REBOOT:
  case ID_CANCEL_REQUEST:
  case ID_GET_CALIBRATION:
    return 0;
  case ID_SET_DEVICE_CONFIG:
    return sizeof(DeviceConfig_t);
  case ID_SET_MAG_CALIBRATON:
    return sizeof(Calibration_t);
  case ID_SET_IMU_CALIBRATON:
    return sizeof(Calibration_t) * 2;
  case ID_DATA_PACKET:
  default:
    return -1;
  }
}

void dispatch_message(const uint8_t *message) {
  Identifiers_t id = (Identifiers_t)message[0];
  switch (id) {
  case ID_GET_DEVICE_INFO:
  case ID_GET_DEVICE_CONFIG:
  case ID_SET_DEVICE_CONFIG:
  case ID_REBOOT:
  case ID_SET_MAG_CALIBRATON:
  case ID_SET_IMU_CALIBRATON:
  case ID_GET_CALIBRATION:
  case ID_CANCEL_REQUEST:
    dispatch_command(message);
    break;
  case ID_DATA_PACKET:
  default:
    break;
  }
}
