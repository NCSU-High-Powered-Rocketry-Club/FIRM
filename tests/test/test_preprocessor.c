#include "unity.h"
#include "stm32_hal_types.h"
#include "preprocessor.h"

void setup(void) {}
void tearDown(void) {}

void bmp581_convert_packet_missing_data(void) {
    BMP581Packet_t packet = {0, 0, 0, 0, 0, 0};
    CalibratedDataPacket_t ret_packet;
    bmp581_convert_packet(&packet, &ret_packet);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.pressure);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.temp);
}

void bmp581_convert_packet_normal_values(void) {
    // ~22.147C, ~100,000.016Pa
    BMP581Packet_t packet = {0b10100011, 0b00100101 ,0b00010110 , 0b00000001, 0b10101000, 0b01100001};
    CalibratedDataPacket_t ret_packet;
    bmp581_convert_packet(&packet, &ret_packet);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 6400001.0F / 64.0F, ret_packet.pressure);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1451427.0F / 65536.0F, ret_packet.temp);
}

void bmp581_convert_packet_negative_values(void) {
    // ~-1.526C, -625Pa
    BMP581Packet_t packet = {0b01100000, 0b01111001 ,0b11111110 , 0b11000000, 0b01100011, 0b11111111};
    CalibratedDataPacket_t ret_packet;
    bmp581_convert_packet(&packet, &ret_packet);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, -40000.0F / 64.0F, ret_packet.pressure);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, -100000.0F / 65536.0F, ret_packet.temp);
}

void mmc5983ma_convert_packet_missing_data(void) {
    MMC5983MAPacket_t packet = {0, 0, 0, 0, 0, 0, 0};
    CalibratedDataPacket_t ret_packet;
    // Assumed function name: mmc5983ma_convert_packet
    mmc5983ma_convert_packet(&packet, &ret_packet);
    // Assumed output field names: mag_x, mag_y, mag_z
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.mag_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.mag_y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.mag_z);
}

void mmc5983ma_convert_packet_normal_values(void) {
    // Construct two sample axes in one test (one positive, one negative)
    // Assumed packing: 18-bit signed value composed as
    // raw = (msb << 10) | (mid << 2) | lsb_bits
    // where lsb_bits for X/Y/Z are in mag_xyz_lsb at bits 7:6, 5:4, 3:2 respectively.
    // Sample values chosen so the computed physical values are round numbers:
    // - X raw = 65536  -> mag_x = 65536 / (131072/800) = 400.0
    // - Y raw = 0      -> mag_y = 0.0
    // - Z raw = -32768 -> mag_z = -200.0
    MMC5983MAPacket_t packet = {0};
    packet.mag_x_msb = 64;   // 65536 >> 10 = 64
    packet.mag_x_mid = 0;
    packet.mag_y_msb = 0;
    packet.mag_y_mid = 0;
    packet.mag_z_msb = 224;  // two's complement 18-bit for -32768 -> (262144 - 32768) >> 10 = 224
    packet.mag_z_mid = 0;
    packet.mag_xyz_lsb = 0;  // low 2 bits for each axis = 0

    CalibratedDataPacket_t ret_packet;
    mmc5983ma_convert_packet(&packet, &ret_packet);

    // scaling factor per your note: divide by (131072.0F/800.0F)
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 400.0F, ret_packet.mag_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.mag_y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, -200.0F, ret_packet.mag_z);
}

void icm45686_convert_packet_missing_data(void) {
    ICM45686Packet_t packet = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    CalibratedDataPacket_t ret_packet;
    // Assumed function name: icm45686_convert_packet
    icm45686_convert_packet(&packet, &ret_packet);
    // Assumed output field names: acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.acc_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.acc_y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.acc_z);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.gyro_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.gyro_y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0F, ret_packet.gyro_z);
}

void icm45686_convert_packet_normal_values(void) {
    // Build a packet with a few positive and negative example values.
    // Assumption: accel and gyro raw are 20-bit signed values. Packing used
    // by production code (as provided) is:
    // acc_binary = (acc_H << 12) | (acc_L << 4) | (vals_lsb >> 4)
    // gyro_binary = (gyro_H << 12) | (gyro_L << 4) | (vals_lsb & 0x0F)
    // Accel scaling: divide by 16384.0F
    // Gyro scaling per your code: divide by (131.072F * (pi/180.0F))

    ICM45686Packet_t packet = {0};
    // For 20-bit packing we choose raw values so expected floats are simple:
    // accel: 16384 -> 1.0, 8192 -> 0.5, -16384 -> -1.0
    // Represented as 20-bit two's complement where needed.
    // acc_x raw = 0x04000 -> accX_H = 0x04, accX_L = 0x00, x_vals_lsb high nibble = 0x0
    packet.accX_H = 0x04; packet.accX_L = 0x00;
    // acc_y raw = 0x02000 -> accY_H = 0x02, accY_L = 0x00
    packet.accY_H = 0x02; packet.accY_L = 0x00;
    // acc_z raw = 20-bit two's complement for -16384 -> 0xFC000
    // accZ_H = 0xFC, accZ_L = 0x00
    packet.accZ_H = 0xFC; packet.accZ_L = 0x00;

    // gyro_x raw = 0x03333 (13107) -> gyroX_H = 0x03, gyroX_L = 0x33, gyro low nibble = 0x3
    packet.gyroX_H = 0x03; packet.gyroX_L = 0x33;
    // gyro_y raw = 20-bit two's complement for -13107 -> 0xFCCCD
    // gyroY_H = 0xFC, gyroY_L = 0xCC, gyro low nibble = 0xD
    packet.gyroY_H = 0xFC; packet.gyroY_L = 0xCC;
    packet.gyroZ_H = 0x00; packet.gyroZ_L = 0x00;

    // Pack x/y/z vals lsb: high nibble = accel low 4 bits (we want 0), low nibble = gyro low 4 bits
    packet.x_vals_lsb = (0x0 << 4) | (0x3 & 0x0F); // 0x03
    packet.y_vals_lsb = (0x0 << 4) | (0xD & 0x0F); // 0x0D
    packet.z_vals_lsb = 0x00;

    CalibratedDataPacket_t ret_packet;
    icm45686_convert_packet(&packet, &ret_packet);

    // expected accel
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0F, ret_packet.acc_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.5F, ret_packet.acc_y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, -1.0F, ret_packet.acc_z);

    // expected gyro (converted to radians/sec using the provided divisor)
    const float pi = 3.14159265358979323846F;
    float expected_gyro_x = 13107.0F / (131.072F * (pi / 180.0F));
    float expected_gyro_y = -13107.0F / (131.072F * (pi / 180.0F));
    float expected_gyro_z = 0.0F;

    TEST_ASSERT_FLOAT_WITHIN(1e-6, expected_gyro_x, ret_packet.gyro_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, expected_gyro_y, ret_packet.gyro_y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, expected_gyro_z, ret_packet.gyro_z);
}

