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

int mag_init(I2C_HandleTypeDef *hi2c) {
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
    result = 0;
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
    i2c_write(hi2c, dev_i2c_addr, internal_control1, 0b00000000);

    // enable continuous measurement mode at 200hz
    i2c_write(hi2c, dev_i2c_addr, internal_control2, 0b00001010);
    return 0;
}

int mag_read(I2C_HandleTypeDef *hi2c, int flip) {
    uint8_t data_ready = 0;
    i2c_read(hi2c, dev_i2c_addr, status, &data_ready, 1);
    if (data_ready & 0x01) {
        if (flip == 0) {
            i2c_write(hi2c, dev_i2c_addr, internal_control0, 0b00010100);
        }
        if (flip == 5) {
            i2c_write(hi2c, dev_i2c_addr, internal_control0, 0b00001100);
        }
        i2c_write(hi2c, dev_i2c_addr, status, 0b00000001);
        uint8_t raw_data[7];
        i2c_read(hi2c, dev_i2c_addr, x_out0, raw_data, 7);
        uint32_t mag_data_binary[3];
        mag_data_binary[0] = (uint32_t)(raw_data[0] << 10 | raw_data[1] << 2 | (raw_data[6] & 0xC0) >> 6);
        mag_data_binary[1] = (uint32_t)(raw_data[2] << 10 | raw_data[3] << 2 | (raw_data[6] & 0x30) >> 4);
        mag_data_binary[2] = (uint32_t)(raw_data[4] << 10 | raw_data[5] << 2 | (raw_data[6] & 0x0C) >> 2);
        float mag_data[3];
        mag_data[0] = (((float)mag_data_binary[0]) - 131072.0) / 131072.0;
        mag_data[1] = (((float)mag_data_binary[1]) - 131072.0) / 131072.0;
        mag_data[2] = (((float)mag_data_binary[2]) - 131072.0) / 131072.0;
        serialPrintFloat(mag_data[0]);
        return 0;
    }
    return 1;
}

