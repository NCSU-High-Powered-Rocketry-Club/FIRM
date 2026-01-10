#pragma once
#include "data_processing/data_preprocess.h"
#include <stdint.h>
#include <stddef.h>

/**
 * Serialized data packet struct for sending sensor and KF data over USB. It is a 136 bytes.
 */
typedef struct {
    uint16_t header;
    uint16_t length;
    DataPacket_t payload;
    uint16_t crc;
} SerializedDataPacket_t;

/**
 * Serialized command response packet struct for sending command responses over USB. To make
 * parsing easier, its the same size as SerializedDataPacket_t:
 * [0xA5 0x5A][LEN(2)][PADDING(4)][PAYLOAD(120)][CRC(2)]
 */
typedef struct {
    uint16_t header;
    uint16_t length;
    uint8_t padding[4]; // SerializedDataPacket_t has 4 bytes of padding here, so to make parsing easier we add it here too
    uint8_t payload[sizeof(DataPacket_t)]; // max payload size is size of data packet
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
 * @param serialized_packet Pointer to the serialized packet that will be written to.
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
 * [0xA5 0x5A][LEN(2)][PADDING(4)][PAYLOAD(120)][CRC(2)]
 * 
 * @param payload Pointer to the payload data.
 * @param payload_len Length of the payload data.
 * @param serialized_packet pointer to the serialized packet that will be written to.
 */
void serialize_command_packet(const uint8_t* payload, uint8_t payload_len, uint8_t* serialized_packet);
