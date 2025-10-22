#include "usb_serializer.h"
#include "usbd_cdc_if.h"

// Return a 64-bit cycle count from DWT
static inline uint64_t dwt_get64(uint32_t dwt_overflow_count) {
    uint32_t high, low;
    do {
        high = dwt_overflow_count;
        low = DWT->CYCCNT;
    } while (low > DWT->CYCCNT);  // protect against rollover

    return ((uint64_t)high << 32) | low;
}

size_t usb_serialize_calibrated_packet(const CalibratedDataPacket_t* packet, uint32_t dwt_overflow_count, uint8_t* out_buf) {
    if (!packet || !out_buf) return 0;

    // Write timestamp directly
    *(uint64_t*)out_buf = dwt_get64(dwt_overflow_count);
    
    // Write packet data directly after timestamp
    *(CalibratedDataPacket_t*)(out_buf + sizeof(uint64_t)) = *packet;

    return sizeof(uint64_t) + sizeof(CalibratedDataPacket_t);
}

void usb_transmit_serialized_packet(const uint8_t* buf, size_t len) {
    if (!buf || len == 0) return;
    CDC_Transmit_FS((uint8_t*)buf, len);  // Cast required by CDC API
}

void usb_serialize_and_send_calibrated_packet(const CalibratedDataPacket_t* packet, uint64_t timestamp) {
    if (!packet) return;

    uint8_t buffer[sizeof(uint64_t) + 9 * sizeof(float)];
    size_t bytes_written = usb_serialize_calibrated_packet(packet, timestamp, buffer);
    if (bytes_written > 0) {
        usb_transmit_serialized_packet(buffer, bytes_written);
    }
}