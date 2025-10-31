#include "unity.h"
#include "preprocessor.h"

void setup(void) {}
void tearDown(void) {}

void test_bmp581_convert_packet_missing_data(void) {
    BMP581Packet_t packet = {0, 0, 0, 0, 0, 0};
    CalibratedDataPacket_t ret_packet;
    bmp581_convert_packet(&packet, &ret_packet);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.pressure);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.temperature);
}

void test_bmp581_convert_packet_normal_values(void) {
    // ~22.147C, ~100,000.016Pa
    BMP581Packet_t packet = {0b10100011, 0b00100101 ,0b00010110 , 0b00000001, 0b10101000, 0b01100001};
    CalibratedDataPacket_t ret_packet;
    bmp581_convert_packet(&packet, &ret_packet);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 6400001.0F / 64.0F, ret_packet.pressure);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1451427.0F / 65536.0F, ret_packet.temperature);
}

void test_bmp581_convert_packet_negative_values(void) {
    // ~-1.526C, -625Pa
    BMP581Packet_t packet = {0b01100000, 0b01111001, 0b11111110, 0b11000000, 0b01100011, 0b11111111};
    CalibratedDataPacket_t ret_packet;
    bmp581_convert_packet(&packet, &ret_packet);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, -100000.0F / 65536.0F, ret_packet.temperature);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, -40000.0F / 64.0F, ret_packet.pressure);
}

void test_mmc5983ma_convert_packet_missing_data(void) {
    MMC5983MAPacket_t packet = {0, 0, 0, 0, 0, 0, 0};
    CalibratedDataPacket_t ret_packet;
    mmc5983ma_convert_packet(&packet, &ret_packet);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.magnetic_field_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.magnetic_field_y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.magnetic_field_z);
}


void test_mmc5983ma_convert_packet_normal_values(void) {
    // X raw = 80000  -> magnetic_field_x = 80000 / (131072/800)  = 488.28125
    // Y raw = 0      -> magnetic_field_y = 0 / (131072/800)      = 0.0
    // Z raw = -41751 -> magnetic_field_z = -41751 / (131072/800) = ~-254.8279
    MMC5983MAPacket_t packet = {0};
    packet.mag_x_msb = 0b01001110;
    packet.mag_x_mid = 0b00100000;
    packet.mag_y_msb = 0;
    packet.mag_y_mid = 0;
    packet.mag_z_msb = 0b11010111;
    packet.mag_z_mid = 0b00111010;
    // 2 bits for each axis (0bxxyyzz00)
    packet.mag_xyz_lsb = 0b00000100;  

    CalibratedDataPacket_t ret_packet;
    mmc5983ma_convert_packet(&packet, &ret_packet);

    TEST_ASSERT_FLOAT_WITHIN(1e-6, 488.28125F, ret_packet.magnetic_field_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.magnetic_field_y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, (-41751.0F / (131072.0F / 800.0F)), ret_packet.magnetic_field_z);
}

void test_icm45686_convert_packet_missing_data(void) {
    ICM45686Packet_t packet = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    CalibratedDataPacket_t ret_packet;
    icm45686_convert_packet(&packet, &ret_packet);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.accel_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.accel_y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.accel_z);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.angular_rate_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.angular_rate_y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.angular_rate_z);
}

void test_icm45686_convert_packet_normal_values(void) {
    const float pi = 3.14159265358979323846F;
    const float acc_scale_factor = 16384.0F;
    const float gyro_scale_factor = (131.072F * (180.0F / pi));

    ICM45686Packet_t packet = {0};
    // X accel raw = 30015   -> accel_x = 30015 / 16384   = ~1.832g
    // Y accel raw = 0       -> accel_y = 0 / 16384       = 0
    // Z accel raw = -524288 -> accel_z = -524288 / 16384 = -32.0
    packet.accX_H = 0x07; packet.accX_L = 0x53;
    packet.accY_H = 0x00; packet.accY_L = 0x00;
    packet.accZ_H = 0x80; packet.accZ_L = 0x00;

    // X gyro raw = 0      -> 0 / (131.072F * (180.0F / pi))                = 0
    // Y gyro raw = -56621 -> -56621 / (131.072F * (180.0F / pi))           = ~7.5395
    // Z gyro raw = 458752 -> accel_z = 458752 / (131.072F * (180.0F / pi)) = ~61.0865
    
    packet.gyroX_H = 0x00; packet.gyroX_L = 0x00;
    packet.gyroY_H = 0xF2; packet.gyroY_L = 0x2D;
    packet.gyroZ_H = 0x70; packet.gyroZ_L = 0x00;

    // 4 bits for accel, 4 bits for gyro (0baaaagggg)
    packet.x_vals_lsb = 0xF0;
    packet.y_vals_lsb = 0x03;
    packet.z_vals_lsb = 0x00;

    CalibratedDataPacket_t ret_packet;
    icm45686_convert_packet(&packet, &ret_packet);

    TEST_ASSERT_FLOAT_WITHIN(1e-6, 30015.0F / acc_scale_factor, ret_packet.accel_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.accel_y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, -32.0F, ret_packet.accel_z);

    float expected_gyro_x = 0.0F;
    float expected_gyro_y = -56621.0F / gyro_scale_factor;
    float expected_gyro_z = 458752.0F / gyro_scale_factor;

    TEST_ASSERT_FLOAT_WITHIN(1e-6, expected_gyro_x, ret_packet.angular_rate_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, expected_gyro_y, ret_packet.angular_rate_y);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, expected_gyro_z, ret_packet.angular_rate_z);
}

