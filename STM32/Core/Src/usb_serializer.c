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

void serialize_command_packet(const uint8_t* payload, uint8_t payload_len, uint8_t* serialized_packet) {
    if (!payload || !serialized_packet) return;

    // This function emits a fixed-size (136-byte) response frame:
    // [0xA5 0x5A][LEN(2)][PADDING(4)][PAYLOAD(120)][CRC(2)][PADDING(6)]
    // CRC is CRC-16-CCITT (KERMIT) over the entire frame excluding the CRC field.
    SerializedResponsePacket_t* response_packet = (SerializedResponsePacket_t*)serialized_packet;
    
    // Clear out the response packet
    memset(response_packet, 0x00, sizeof(SerializedResponsePacket_t));

    // Fill in header and fixed length
    response_packet->header = 0x5AA5; // little-endian -> A5 5A
    if (payload_len > sizeof(response_packet->payload)) {
        payload_len = (uint8_t)sizeof(response_packet->payload);
    }
    response_packet->length = sizeof(DataPacket_t);

    // padding is already zeroed by memset, this copies the payload to the packet
    memcpy(response_packet->payload, payload, payload_len);

    // Calculate CRC over the entire packet except the CRC itself
    const uint16_t calc_len = (uint16_t)offsetof(SerializedResponsePacket_t, crc);
    response_packet->crc = crc16_ccitt(serialized_packet, calc_len);
}
