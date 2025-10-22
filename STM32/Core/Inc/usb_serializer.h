#pragma once
#include "usb_print_debug.h"
#include <stdint.h>
#include <stddef.h>


/**
 * Calibrated data packet (all floats)
 */
typedef struct CalibratedDataPacket_t {
	float accel_x;
	float accel_y;
	float accel_z;
	float angular_rate_x;
	float angular_rate_y;
	float angular_rate_z;
	float magnetic_field_x;
	float magnetic_field_y;
	float magnetic_field_z;
} CalibratedDataPacket_t;


/**
 * @brief Serialize a calibrated data packet into a provided buffer. The serialization
 * format is fixed: 8-byte little-endian timestamp followed by nine 32-bit floats
 * in this order: accel_x,y,z; angular_rate_x,y,z; magnetic_field_x,y,z.
 *
 * @param packet Pointer to the calibrated packet to serialize.
 * @param timestamp 64-bit timestamp to include at the start of the serialized packet.
 * @param out_buf Buffer to write the serialized bytes into. Caller must ensure it
 *                has at least 8 + 9*sizeof(float) bytes available.
 * @return number of bytes written to out_buf (always 8 + 9*sizeof(float) on success),
 *         or 0 on error (e.g., null pointers).
 */
size_t usb_serialize_calibrated_packet(const CalibratedDataPacket_t* packet, uint32_t dwt_overflow_count, uint8_t* out_buf);

/**
 * @brief Transmit a previously serialized buffer over USB CDC.
 * @param buf pointer to serialized data
 * @param len length in bytes
 */
void usb_transmit_serialized_packet(const uint8_t* buf, size_t len);

/**
 * @brief Convenience wrapper: serialize `packet` into an internal buffer and transmit it over USB.
 */
void usb_serialize_and_send_calibrated_packet(const CalibratedDataPacket_t* packet, uint64_t timestamp);
