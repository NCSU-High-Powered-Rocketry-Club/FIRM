#pragma once
#include "shared_data/packets.h"
#include <stdint.h>

#define MAX_TRANSMIT_PAYLOAD_LEN 200

/**
 * @brief Dataframe used to send to transmit task, containing the size and data
 */
typedef struct __attribute__((packed)) {
  uint16_t payload_len;
  uint8_t payload[MAX_TRANSMIT_PAYLOAD_LEN];
} TransmitFrame_t;