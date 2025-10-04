/*
 * mmc5983ma.c
 *
 *  Created on: Sep 8, 2025
 *      Author: Wlsan
 */
#include "mmc5983ma.h"

const uint8_t dev_i2c_addr = 0x30; // the magnetometer i2c address

// magnetometer register mapping
const uint8_t x_out0 = 0x00;
const uint8_t status = 0x08;
const uint8_t internal_control0 = 0x09;
const uint8_t internal_control1 = 0x0A;
const uint8_t internal_control2 = 0x0B;
const uint8_t product_id1 = 0x2F;

const uint8_t product_id_val = 0x30; // expected value for the product ID register
// value to divide shifted mag value by to get result in microtesla (SI Units)
const float scaling_factor = 131072.0 / 800.0;
const int flip_interval = 10; // number of regular packets between a flipped-sign packet

int magnetometer_init(I2C_HandleTypeDef* hi2c) {
    HAL_Delay(14); // 15ms power-on time
    uint8_t result = 0;
    // dummy read, ignore result
    if (i2c_read(hi2c, dev_i2c_addr, product_id1, &result, 1) == HAL_ERROR) {
        serialPrintStr("I2C read failed HAL ERROR");
        return 1;
    }
    // read product ID, check that result is the expected Product ID byte
    result = 0;
    i2c_read(hi2c, dev_i2c_addr, product_id1, &result, 1);
    if (result != product_id_val) {
        serialPrintStr("MMC5983MA could not read Product ID");
        return 1;
    }
    // initiating a software reset
    i2c_write(hi2c, dev_i2c_addr, internal_control1, 0b10000000);
    HAL_Delay(14); // 15ms power-on time
    // do dummy read after reset
    i2c_read(hi2c, dev_i2c_addr, product_id1, &result, 1);
    // read product ID, check that result is the expected Product ID byte
    i2c_read(hi2c, dev_i2c_addr, product_id1, &result, 1);
    if (result != product_id_val) {
        serialPrintStr("MMC5983MA could not read Product ID after reset");
        return 1;
    }

    // check that bit 7 (sw_rst) is back to 0
    i2c_read(hi2c, dev_i2c_addr, internal_control1, &result, 1);
    if (result & 0x80) {
        serialPrintStr("MMC5983MA could not complete software reset");
        return 1;
    }
    // enable interrupt pin
    i2c_write(hi2c, dev_i2c_addr, internal_control0, 0b00000100);
    // set bandwidth to 200hz (4ms measurement time)
    i2c_write(hi2c, dev_i2c_addr, internal_control1, 0b00000001);
    // enable continuous measurement mode at 200hz
    i2c_write(hi2c, dev_i2c_addr, internal_control2, 0b00001110);
    return 0;
}

int magnetometer_read(I2C_HandleTypeDef* hi2c, MagnetometerPacket_t* packet, uint8_t* flip) {
    uint8_t data_ready = 0;
    // read status register to make sure data is ready
    i2c_read(hi2c, dev_i2c_addr, status, &data_ready, 1);
    if (data_ready & 0x01) { // data ready bit is 0x01
        // manually clear the interrupt signal
        i2c_write(hi2c, dev_i2c_addr, status, 0b00000001);
        uint8_t raw_data[7];
        uint32_t mag_data_binary[3];
        float mag_data[3];
        // every flip_interval read cycles, flip the polarity of the magnetometer values
        // to calibrate the sensor properly
        if (*flip % flip_interval == 0) {
            i2c_write(hi2c, dev_i2c_addr, internal_control0, 0b00010100);
        }
        if ((*flip - 1) % flip_interval == 0) {
            i2c_write(hi2c, dev_i2c_addr, internal_control0, 0b00001100);
        }

        i2c_read(hi2c, dev_i2c_addr, x_out0, raw_data, 7);
        mag_data_binary[0] =
            (uint32_t)(raw_data[0] << 10 | raw_data[1] << 2 | (raw_data[6] & 0xC0) >> 6);
        mag_data_binary[1] =
            (uint32_t)(raw_data[2] << 10 | raw_data[3] << 2 | (raw_data[6] & 0x30) >> 4);
        mag_data_binary[2] =
            (uint32_t)(raw_data[4] << 10 | raw_data[5] << 2 | (raw_data[6] & 0x0C) >> 2);
        mag_data[0] = (((float)mag_data_binary[0]) - 131072.0) / scaling_factor;
        mag_data[1] = (((float)mag_data_binary[1]) - 131072.0) / scaling_factor;
        mag_data[2] = (((float)mag_data_binary[2]) - 131072.0) / scaling_factor;
        packet->mag_x = mag_data[0];
        packet->mag_y = mag_data[1];
        packet->mag_z = mag_data[2];
        (*flip)++; // incrememt the flip counter
        return 0;
    }
    return 1;
}
