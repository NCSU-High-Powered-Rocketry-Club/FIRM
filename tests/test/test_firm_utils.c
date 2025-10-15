#include "unity.h"
#include "firm_utils.h"

void setup(void) {}
void tearDown(void) {}

void test_twos_complement_16_should_return_positive_values(void)
{
    // 0x1234 = 4660
    TEST_ASSERT_EQUAL_INT16(4660, twos_complement_16(0x12, 0x34));

    // 0x7FFF = 32767 (largest positive signed 16-bit)
    TEST_ASSERT_EQUAL_INT16(32767, twos_complement_16(0x7F, 0xFF));
}

void test_twos_complement_16_should_return_negative_values(void)
{
    // 0xFFFF = -1
    TEST_ASSERT_EQUAL_INT16(-1, twos_complement_16(0xFF, 0xFF));

    // 0x8000 = -32768 (most negative 16-bit)
    TEST_ASSERT_EQUAL_INT16(-32768, twos_complement_16(0x80, 0x00));
}

void test_sign_extend_20bit_should_sign_extend_correctly(void)
{
    // 0x00000 = 0
    TEST_ASSERT_EQUAL_INT32(0, sign_extend_20bit(0x00000));

    // 0x7FFFF = 524287 (largest positive 20-bit)
    TEST_ASSERT_EQUAL_INT32(524287, sign_extend_20bit(0x7FFFF));

    // 0x80000 = -524288 (smallest negative 20-bit)
    TEST_ASSERT_EQUAL_INT32(-524288, sign_extend_20bit(0x80000));

    // 0xABCDE = -216210 (random negative 20-bit)
    TEST_ASSERT_EQUAL_INT32(-216210, sign_extend_20bit(0xABCDE));
}