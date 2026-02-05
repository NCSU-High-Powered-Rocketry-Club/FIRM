#include "unity.h"

#include <string.h>

#include "messages.h"
#include "utils.h"

static uint16_t compute_expected_crc(uint16_t header,
                                   uint16_t identifier,
                                   uint32_t payload_length,
                                   const uint8_t *payload) {
    uint8_t scratch[2 + 2 + 4 + 64];

    TEST_ASSERT_TRUE(payload_length <= 64);

    memcpy(&scratch[0], &header, sizeof(header));
    memcpy(&scratch[sizeof(header)], &identifier, sizeof(identifier));
    memcpy(&scratch[sizeof(header) + sizeof(identifier)], &payload_length, sizeof(payload_length));
    if (payload_length > 0) {
        memcpy(&scratch[sizeof(header) + sizeof(identifier) + sizeof(payload_length)], payload, payload_length);
    }

    return crc16_ccitt(scratch, (uint32_t)(sizeof(header) + sizeof(identifier) + sizeof(payload_length) + payload_length));
}

void setUp(void) {
    // no-op
}

void tearDown(void) {
    // no-op
}

void test_usb_parse_message_meta_rejects_short_and_parses_valid(void) {
    UsbMessageMeta meta;

    const uint8_t too_short[] = {0};
    TEST_ASSERT_FALSE(usb_parse_message_meta(too_short, sizeof(too_short), &meta));

    uint8_t meta_bytes[sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint32_t)] = {0};
    uint16_t header = MSGID_COMMAND_PACKET;
    uint16_t identifier = CMDID_GET_DEVICE_INFO;
    uint32_t payload_len = 0;

    memcpy(&meta_bytes[0], &header, sizeof(header));
    memcpy(&meta_bytes[sizeof(header)], &identifier, sizeof(identifier));
    memcpy(&meta_bytes[sizeof(header) + sizeof(identifier)], &payload_len, sizeof(payload_len));

    TEST_ASSERT_TRUE(usb_parse_message_meta(meta_bytes, sizeof(meta_bytes), &meta));
    TEST_ASSERT_EQUAL_UINT16(MSGID_COMMAND_PACKET, meta.header);
    TEST_ASSERT_EQUAL_UINT16(CMDID_GET_DEVICE_INFO, meta.identifier);
    TEST_ASSERT_EQUAL_UINT32(0, meta.payload_length);
}

void test_usb_interpret_usb_message_command_valid_crc(void) {
    UsbMessageMeta meta = {
        .header = MSGID_COMMAND_PACKET,
        .identifier = CMDID_GET_DEVICE_INFO,
        .payload_length = 0,
    };

    uint8_t payload_and_crc[2] = {0};
    uint16_t crc = compute_expected_crc(meta.header, meta.identifier, meta.payload_length, NULL);
    payload_and_crc[0] = (uint8_t)(crc & 0xFFu);
    payload_and_crc[1] = (uint8_t)((crc >> 8) & 0xFFu);

    TEST_ASSERT_EQUAL(USBMSG_COMMAND, usb_interpret_usb_message(&meta, payload_and_crc, sizeof(payload_and_crc)));
}

void test_usb_interpret_usb_message_mock_sensor_ignores_crc(void) {
    UsbMessageMeta meta = {
        .header = MSGID_MOCK_PACKET,
        .identifier = (uint16_t)'B',
        .payload_length = 3,
    };

    // CRC bytes are intentionally wrong; mock sensor packets should still be accepted.
    uint8_t payload_and_crc[3 + 2] = {0xAA, 0xBB, 0xCC, 0x00, 0x00};

    TEST_ASSERT_EQUAL(USBMSG_MOCK_SENSOR,
                      usb_interpret_usb_message(&meta, payload_and_crc, sizeof(payload_and_crc)));
}
