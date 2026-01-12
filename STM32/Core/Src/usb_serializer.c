#include "usb_serializer.h"
#include "usbd_cdc_if.h"
#include "commands.h"
#include "utils.h"
#include <string.h>


void serialize_packet(uint8_t *payload, uint16_t packet_len) {
  if (!payload || packet_len == 0) return;
  // calculate and set the crc bytes based on CRC-ccitt-KERMIT format
  payload[packet_len] = crc16_ccitt(payload, packet_len);
}