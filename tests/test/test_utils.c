#include "unity.h"
#include "utils.h"

void test_clamp_u16(void) {
    TEST_ASSERT_EQUAL_UINT16(10, clamp_u16(1, 10, 100));
    TEST_ASSERT_EQUAL_UINT16(10, clamp_u16(10, 10, 100));
    TEST_ASSERT_EQUAL_UINT16(50, clamp_u16(50, 10, 100));
    TEST_ASSERT_EQUAL_UINT16(100, clamp_u16(100, 10, 100));
    TEST_ASSERT_EQUAL_UINT16(100, clamp_u16(101, 10, 100));
}

void test_crc16_ccitt(void) {
    const uint8_t data1[] = {0x31, 0x32, 0x33, 0x34, 0x35};
    TEST_ASSERT_EQUAL_UINT16(29751, crc16_ccitt(data1, sizeof(data1)));

    const uint8_t data2[] = {};
    TEST_ASSERT_EQUAL_UINT16(0, crc16_ccitt(data2, sizeof(data2)));

    const uint8_t data3[] = {0xAA, 0x61, 0x33, 0x34, 0xDE, 0xAD, 0xBE, 0xEF};
    TEST_ASSERT_EQUAL_UINT16(19432, crc16_ccitt(data3, sizeof(data3)));
}
