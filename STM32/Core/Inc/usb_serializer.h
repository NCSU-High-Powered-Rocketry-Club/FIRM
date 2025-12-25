#pragma once
#include "data_processing/preprocessor.h"
#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint16_t header;
    uint16_t length;
    DataPacket_t payload;
    uint16_t crc;
} SerializedDataPacket_t;

typedef struct {
    uint16_t header;
    uint16_t length;
    uint8_t  padding[4];
    uint8_t  payload[56];
    uint16_t crc;
} SerializedResponsePacket_t;

/**
 * @brief initializes a serialized data packet with header and length fields
 * @param serialized_packet pointer to a SerializedDataPacket_t, the packet to initialize
 */
void serializer_init_data_packet(SerializedDataPacket_t *serialized_packet);

/**
 * @brief Serialize a data packet into a provided buffer. The serialization
 * format is fixed: 8-byte little-endian timestamp followed by nine 32-bit floats
 * in this order: accel_x,y,z; angular_rate_x,y,z; magnetic_field_x,y,z.
 *
 * @param packet Pointer to the data packet to serialize.
 * @param serialized_packet pointer to the serialized packet that will be written to
 * @return number of bytes written to out_buf (always 8 + 9*sizeof(float) on success),
 *         or 0 on error (e.g., null pointers).
 */
void serialize_data_packet(const DataPacket_t* packet, SerializedDataPacket_t *serialized_packet);

/**
 * @brief Transmit a serialized data packet over USB CDC.
 * @param serialized_packet pointer to serialized data packet
 */
void usb_transmit_serialized_packet(const SerializedDataPacket_t *serialized_packet);

/**
 * @brief Serializes a command response payload into a full response packet matching the
 * Rust-side SerialParser format:
 * [0xA5 0x5A][LEN(2)=56][PADDING(4)][PAYLOAD(56)][CRC(2)]
 * 
 * @param payload Pointer to the payload data.
 * @param payload_len Length of the payload data.
 * @param out_packet Buffer to store the final serialized packet (must be at least 66 bytes).
 */
void serialize_command_packet(const uint8_t* payload, uint8_t payload_len, uint8_t* out_packet);
