#pragma once
#include "preprocessor.h"
#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint16_t header;
    uint16_t length;
    CalibratedDataPacket_t payload;
    uint16_t crc;
} SerializedPacket_t;

/**
 * @brief initializes a serialized data packet with header and length fields
 * @param serialized_packet pointer to a SerializedPacket_t, the packet to initialize
 */
void serializer_init_packet(SerializedPacket_t *serialized_packet);

/**
 * @brief Serialize a calibrated data packet into a provided buffer. The serialization
 * format is fixed: 8-byte little-endian timestamp followed by nine 32-bit floats
 * in this order: accel_x,y,z; angular_rate_x,y,z; magnetic_field_x,y,z.
 *
 * @param packet Pointer to the calibrated packet to serialize.
 * @param serialized_packet pointer to the serialized packet that will be written to
 * @return number of bytes written to out_buf (always 8 + 9*sizeof(float) on success),
 *         or 0 on error (e.g., null pointers).
 */
void usb_serialize_calibrated_packet(const CalibratedDataPacket_t* packet, SerializedPacket_t *serialized_packet);

/**
 * @brief Transmit a serialized data packet over USB CDC.
 * @param serialized_packet pointer to serialized data packet
 */
void usb_transmit_serialized_packet(const SerializedPacket_t *serialized_packet);

/**
 * Compute CRC-16-CCITT (KERMIT) over the data buffer.
 * @param data Pointer to input data (excluding CRC).
 * @param len Length in bytes.
 * @return 16-bit CRC (transmit LSB first).
 */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);
