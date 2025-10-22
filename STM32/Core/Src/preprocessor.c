#include "bmp581.h"
#include "mmc5983ma.h"
#include "icm45686.h"

static const float pi = 3.141592653F;

void bmp581_convert_packet(BMP581Packet_t *packet){
    int32_t temp_binary, pressure_binary;

    temp_binary = (((int32_t) (*packet).temp_msb) << 16 )| (((int32_t) (*packet).temp_lsb) << 8) | ((int32_t) (*packet).temp_xlsb);
    pressure_binary = (((int32_t) (*packet).pressure_msb)  << 16) | (((int32_t) (*packet).pressure_lsb) << 8) | ((int32_t) (*packet).pressure_xlsb);

    float temp_float = (float) temp_binary/65536.0F;

    float pressure_float = (float) pressure_binary/64.0F;
}

void mmc5983_convert_packet(MMC5983MAPacket_t *packet1){

    int32_t mag_binary_x,mag_binary_y,mag_binary_z;

    mag_binary_x = (((int32_t) (*packet1).mag_x_msb) << 10 ) | (((int32_t) (*packet1).mag_x_mid) << 2) | ((int32_t) ((*packet1).mag_xyz_lsb >> 6));
    mag_binary_y = (((int32_t) (*packet1).mag_y_msb) << 10 ) | (((int32_t) (*packet1).mag_y_mid) << 2) | ((int32_t) (((*packet1).mag_xyz_lsb & 00110000) >> 4));
    mag_binary_z = (((int32_t) (*packet1).mag_z_msb) << 10 ) | (((int32_t) (*packet1).mag_z_mid) << 2) | ((int32_t) (((*packet1).mag_xyz_lsb & 00001100 >> 2)));
    
    float mag_float_x = ((float) mag_binary_x) / (131072.0F/800.0F);
    float mag_float_y = ((float) mag_binary_y) / (131072.0F/800.0F);
    float mag_float_z = ((float) mag_binary_z) / (131072.0F/800.0F);

}

void imu_convert_packet(ICM45686Packet_t *packet3){
    int32_t acc_binary_x, acc_binary_y, acc_binary_z, gyro_binary_x,gyro_binary_y,gyro_binary_z;

    acc_binary_x = (((int32_t) (*packet3).accX_H) << 12 ) | (((int32_t) (*packet3).accX_L) << 4) | ((int32_t) ((*packet3).x_vals_lsb >> 4));
    acc_binary_y = (((int32_t) (*packet3).accY_H) << 12 ) | (((int32_t) (*packet3).accY_L) << 4) | ((int32_t) ((*packet3).y_vals_lsb >> 4));
    acc_binary_z = (((int32_t) (*packet3).accZ_H) << 12 ) | (((int32_t) (*packet3).accZ_L) << 4) | ((int32_t) ((*packet3).z_vals_lsb >> 4));

    gyro_binary_x = (((int32_t) (*packet3).gyroX_H) << 12) | (((int32_t) (*packet3).gyroX_L) << 4) | ((int32_t) ((*packet3).x_vals_lsb & 00001111));
    gyro_binary_y = (((int32_t) (*packet3).gyroY_H) << 12) | (((int32_t) (*packet3).gyroY_L) << 4) | ((int32_t) ((*packet3).y_vals_lsb & 00001111));
    gyro_binary_z = (((int32_t) (*packet3).gyroZ_H) << 12) | (((int32_t) (*packet3).gyroZ_L) << 4) | ((int32_t) ((*packet3).z_vals_lsb & 00001111));

    float acc_float_x = ((float) acc_binary_x ) / 16384.0F;
    float acc_float_y = ((float) acc_binary_y ) / 16384.0F;
    float acc_float_z = ((float) acc_binary_z ) / 16384.0F;

    float gryo_float_x = ((float) gyro_binary_x) / (131.072F * (pi/180.0F));
    float gryo_float_y = ((float) gyro_binary_y) / (131.072F * (pi/180.0F));
    float gryo_float_z =((float) gyro_binary_z) / (131.072F * (pi/180.0F));
} 

