#include "preprocessor.h"

static const float pi = 3.141592653F;
static uint32_t dwt_overflow_count = 0;
static uint32_t last_cyccnt = 0;

/**
 * @brief Updates dwt overflow counter and returns current timestamp
 * @note Called frequently enough that we can't miss an overflow.
 * DWT->CYCCNT overflows every ~25 seconds at 168MHz.
 * 
 * @return Current timestamp as a double
 */
static double update_dwt_timestamp(void);

void bmp581_convert_packet(BMP581Packet_t *packet, CalibratedDataPacket_t *result_packet){
    // get the current timestamp of the packet in seconds using the DWT counter
    result_packet->timestamp_sec = update_dwt_timestamp();

    int32_t temp_binary, pressure_binary;

    temp_binary = (((int32_t) (*packet).temp_msb) << 16 )| (((int32_t) (*packet).temp_lsb) << 8) | ((int32_t) (*packet).temp_xlsb);
    pressure_binary = (((int32_t) (*packet).pressure_msb)  << 16) | (((int32_t) (*packet).pressure_lsb) << 8) | ((int32_t) (*packet).pressure_xlsb);

    float temp_float = (float) temp_binary/65536.0F;

    float pressure_float = (float) pressure_binary/64.0F;

    result_packet -> temperature = temp_float;
    result_packet -> pressure = pressure_float;
}

void mmc5983ma_convert_packet(MMC5983MAPacket_t *packet, CalibratedDataPacket_t *result_packet){
    // get the current timestamp of the packet in seconds using the DWT counter
    result_packet->timestamp_sec = update_dwt_timestamp();
    int32_t mag_binary_x, mag_binary_y, mag_binary_z;

    mag_binary_x = (((int32_t) (*packet).mag_x_msb) << 10 ) | (((int32_t) (*packet).mag_x_mid) << 2) | ((int32_t) ((*packet).mag_xyz_lsb >> 6));
    mag_binary_y = (((int32_t) (*packet).mag_y_msb) << 10 ) | (((int32_t) (*packet).mag_y_mid) << 2) | ((int32_t) (((*packet).mag_xyz_lsb & 00110000) >> 4));
    mag_binary_z = (((int32_t) (*packet).mag_z_msb) << 10 ) | (((int32_t) (*packet).mag_z_mid) << 2) | ((int32_t) (((*packet).mag_xyz_lsb & 00001100 >> 2)));
    
    float mag_float_x = ((float) mag_binary_x) / (131072.0F/800.0F);
    float mag_float_y = ((float) mag_binary_y) / (131072.0F/800.0F);
    float mag_float_z = ((float) mag_binary_z) / (131072.0F/800.0F);

    result_packet -> magnetic_field_x = mag_float_x;
    result_packet -> magnetic_field_y = mag_float_y;
    result_packet -> magnetic_field_z = mag_float_z;
}

void icm45686_convert_packet(ICM45686Packet_t *packet, CalibratedDataPacket_t *result_packet){
    // get the current timestamp of the packet in seconds using the DWT counter
    result_packet->timestamp_sec = update_dwt_timestamp();
    int32_t acc_binary_x, acc_binary_y, acc_binary_z, gyro_binary_x,gyro_binary_y,gyro_binary_z;

    acc_binary_x = (((int32_t) (*packet).accX_H) << 12 ) | (((int32_t) (*packet).accX_L) << 4) | ((int32_t) ((*packet).x_vals_lsb >> 4));
    acc_binary_y = (((int32_t) (*packet).accY_H) << 12 ) | (((int32_t) (*packet).accY_L) << 4) | ((int32_t) ((*packet).y_vals_lsb >> 4));
    acc_binary_z = (((int32_t) (*packet).accZ_H) << 12 ) | (((int32_t) (*packet).accZ_L) << 4) | ((int32_t) ((*packet).z_vals_lsb >> 4));

    gyro_binary_x = (((int32_t) (*packet).gyroX_H) << 12) | (((int32_t) (*packet).gyroX_L) << 4) | ((int32_t) ((*packet).x_vals_lsb & 00001111));
    gyro_binary_y = (((int32_t) (*packet).gyroY_H) << 12) | (((int32_t) (*packet).gyroY_L) << 4) | ((int32_t) ((*packet).y_vals_lsb & 00001111));
    gyro_binary_z = (((int32_t) (*packet).gyroZ_H) << 12) | (((int32_t) (*packet).gyroZ_L) << 4) | ((int32_t) ((*packet).z_vals_lsb & 00001111));

    float acc_float_x = ((float) acc_binary_x ) / 16384.0F;
    float acc_float_y = ((float) acc_binary_y ) / 16384.0F;
    float acc_float_z = ((float) acc_binary_z ) / 16384.0F;

    float gyro_float_x = ((float) gyro_binary_x) / (131.072F * (pi/180.0F));
    float gyro_float_y = ((float) gyro_binary_y) / (131.072F * (pi/180.0F));
    float gyro_float_z =((float) gyro_binary_z) / (131.072F * (pi/180.0F));

    result_packet -> accel_x = acc_float_x;
    result_packet -> accel_y = acc_float_y;
    result_packet -> accel_z = acc_float_z;

    result_packet -> angular_rate_x = gyro_float_x;
    result_packet -> angular_rate_y = gyro_float_y;
    result_packet -> angular_rate_z = gyro_float_z;
    
} 

static double update_dwt_timestamp(void) {
    uint32_t current_cyccnt = DWT->CYCCNT;
    // Check for overflow by comparing with last value
    // Overflow occurred if current value is less than last value
    if (current_cyccnt < last_cyccnt) {
        dwt_overflow_count++;
    }
    last_cyccnt = current_cyccnt;
    uint64_t cycle_count = ((uint64_t)dwt_overflow_count << 32) | current_cyccnt;
    return ((double)cycle_count) / 168000000.0F;
}