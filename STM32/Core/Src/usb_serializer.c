#include "usb_serializer.h"
#include "usbd_cdc_if.h"
#include "commands.h"
#include "utils.h"
#include <string.h>

void serializer_init_data_packet(SerializedDataPacket_t *serialized_packet) {
    // random header I chose, no significance behind this. It is used as a start value
    // so that the decoder knows where the packet starts
    serialized_packet->header = 0xA55A;
    serialized_packet->length = sizeof(DataPacket_t);
}

void serialize_data_packet(const DataPacket_t *packet, SerializedDataPacket_t *serialized_packet) {
    if (!packet || !serialized_packet) return;
    // header bytes and/or length bytes not properly initialized
    if (!serialized_packet->header || !serialized_packet->length) return;

    serialized_packet->payload = *packet;
    // size of the data minus the crc, theres 4 bytes of padding in the struct between the
    // length field and the data packet field due to the data packet having a double
    uint16_t data_len = offsetof(SerializedDataPacket_t, crc);
    const uint8_t *data = (const uint8_t *)serialized_packet;
    // calculate and set the crc bytes based on CRC-ccitt-KERMIT format
    serialized_packet->crc = crc16_ccitt(data, data_len);
}

void usb_transmit_serialized_packet(const SerializedDataPacket_t *serialized_packet) {
    if (!serialized_packet->crc) return;
    // uint16_t cast is required by CDC API
    
    CDC_Transmit_FS((uint8_t*)serialized_packet, (uint16_t)sizeof(SerializedDataPacket_t));
}

void serialize_command_packet(const uint8_t* payload, uint8_t payload_len, uint8_t* out_packet) {
    if (!payload || !out_packet) return;

    // This function emits a 66-byte response frame:
    // [0xA5 0x5A][LEN(2)=56][PADDING(4)][PAYLOAD(56)][CRC(2)]
    // CRC is CRC-16-CCITT (KERMIT) over the first 64 bytes.
    SerializedResponsePacket_t response_packet;
    memset(&response_packet, 0x00, sizeof(response_packet));

    // Initialize response wrapper (same pattern as serializer_init_data_packet).
    // Header bytes for responses: A5 5A
    // Stored little-endian as 0x5AA5
    response_packet.header = 0x5AA5;
    response_packet.length = sizeof(response_packet.payload);
    memset(response_packet.padding, 0x00, sizeof(response_packet.padding));

    uint8_t actual_len = payload_len;
    if (actual_len > sizeof(response_packet.payload)) {
        actual_len = sizeof(response_packet.payload);
    }
    memcpy(response_packet.payload, payload, actual_len);

    const uint16_t data_len = (uint16_t)offsetof(SerializedResponsePacket_t, crc);
    const uint8_t* data = (const uint8_t*)&response_packet;
    response_packet.crc = crc16_ccitt(data, data_len);

    memcpy(out_packet, &response_packet, sizeof(response_packet));
}
