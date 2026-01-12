#pragma once
#include "data_processing/data_preprocess.h"
#include <stdint.h>
#include <stddef.h>



/**
 * @brief Serialize a data packet into a provided buffer. The serialization
 * format is fixed: 8-byte little-endian timestamp followed by nine 32-bit floats
 * in this order: accel_x,y,z; angular_rate_x,y,z; magnetic_field_x,y,z.
 *
 * @param packet Pointer to the data packet to serialize.
 * @param serialized_packet Pointer to the serialized packet that will be written to.
 * @return number of bytes written to out_buf (always 8 + 9*sizeof(float) on success),
 *         or 0 on error (e.g., null pointers).
 */
void serialize_packet(uint8_t *payload, uint16_t packet_len);
