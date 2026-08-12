#pragma once
#include "shared_data/packets.h"
#include <stdint.h>

#define MAX_TRANSMIT_PAYLOAD_LEN 200

/** Internal queue item. payload_len controls the USB write and is not sent on the wire. */
typedef struct __attribute__((packed)) {
  uint16_t payload_len;
  uint8_t payload[MAX_TRANSMIT_PAYLOAD_LEN];
} TransmitFrame_t;
