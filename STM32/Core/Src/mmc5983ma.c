/*
 * mmc5983ma.c
 *
 *  Created on: Sep 8, 2025
 *      Author: Wlsan
 */
#include "mmc5983ma.h"

/**
 * @brief Starts up and resets the magnetometer, confirms the I2C read/write functionality is working
 *
 * @param soft_reset_complete if this is a setup after a soft reset is complete
 * @retval 0 if successful
 */
static int mag_setup_device(bool soft_reset_complete);

/**
 * @brief Reads data from the MMC5983MA with I2C
 *
 * @param reg_addr the address of the register
 * @param buffer where the result of the read will be stored
 * @param len the number of bytes to read
 * @retval HAL Status, 0 on successful read
 */
static HAL_StatusTypeDef mag_i2c_read(uint8_t reg_addr, uint8_t* buffer, size_t len);

/**
 * @brief Writes 1 byte of data to the MMC5983MA with I2C
 *
 * @param reg_addr the address of the register
 * @param data the data to write to the register
 * @retval HAL Status, 0 on successful write
 */
static HAL_StatusTypeDef mag_i2c_write(uint8_t reg_addr, uint8_t data);

// magnetometer register mapping
static const uint8_t x_out0 = 0x00;
static const uint8_t status = 0x08;
static const uint8_t internal_control0 = 0x09;
static const uint8_t internal_control1 = 0x0A;
static const uint8_t internal_control2 = 0x0B;
static const uint8_t product_id1 = 0x2F;
static const uint8_t product_id_val = 0x30; // expected value for the product ID register

// number of LSBs in the 18-bit data
static const int data_num_lsb_bits = 131072;
// value to divide the data by to convert the magnetic field readings to microtesla
static const float scale_factor = (float)data_num_lsb_bits / 800.0F;
static const int flip_interval = 10; // number of regular packets between a flipped-sign packet

static MagI2CSettings I2CSettings;

int mag_init(I2C_HandleTypeDef* hi2c, uint8_t device_i2c_addr) {
    if (hi2c == NULL) {
        serialPrintStr("Invalid i2c handle for MMC5983MA");
        return 1;
    }

    // configure i2c settings
    I2CSettings.hi2c = hi2c;
    I2CSettings.dev_addr = device_i2c_addr;
    serialPrintStr("Beginning MMC5983MA initialization");

    // sets up the magnetometer in I2C mode and ensures I2C is working
    if (mag_setup_device(false)) return 1;

    // initiating a software reset
    serialPrintStr("\tIssuing MMC5983MA software reset...");
    mag_i2c_write(internal_control1, 0b10000000);

    // verify correct setup again
    if (mag_setup_device(true)) return 1;

    // enable interrupt pin
    mag_i2c_write(internal_control0, 0b00000100);
    // set bandwidth to 200hz (4ms measurement time)
    mag_i2c_write(internal_control1, 0b00000001);
    // enable continuous measurement mode at 200hz
    mag_i2c_write(internal_control2, 0b00001110);

    serialPrintStr("\tMMC5983MA startup successful!");
    return 0;
}

int mag_read_data(MMCPacket_t* packet, uint8_t* flip) {
    uint8_t data_ready = 0;
    // read status register to make sure data is ready
    mag_i2c_read(status, &data_ready, 1);
    if (data_ready & 0x01) { // data ready bit is 0x01
        // manually clear the interrupt signal
        mag_i2c_write(status, 0b00000001);
        uint8_t raw_data[7];
        uint32_t mag_data_binary[3];
        float mag_data[3];
        // every flip_interval read cycles, flip the polarity of the magnetometer values
        // to calibrate the sensor properly
        if (*flip % flip_interval == 0) {
            mag_i2c_write(internal_control0, 0b00010100);
        }
        if ((*flip - 1) % flip_interval == 0) {
            mag_i2c_write(internal_control0, 0b00001100);
        }

        // burst read 7 bytes of data from the first data register
        // first two bytes are magnetometer x
        // next two are magnetometer y
        // next two are magnetometer z
        // last byte is 2 bits of LSB x, 2 bits of LSB y, 2 bits of LSB z, and 2 reserved bits
        mag_i2c_read(x_out0, raw_data, 7);
        mag_data_binary[0] =
            (uint32_t)(raw_data[0] << 10 | raw_data[1] << 2 | (raw_data[6] & 0xC0) >> 6);
        mag_data_binary[1] =
            (uint32_t)(raw_data[2] << 10 | raw_data[3] << 2 | (raw_data[6] & 0x30) >> 4);
        mag_data_binary[2] =
            (uint32_t)(raw_data[4] << 10 | raw_data[5] << 2 | (raw_data[6] & 0x0C) >> 2);
        // the data must be shifted by half of the FS range (262,144 bits).
        // by default the readings are from 0 to 262144, with a magnetic field value of zero
        // being 131072. To get the data centered, we must subtract by 131072.
        mag_data[0] = (float)((mag_data_binary[0]) - data_num_lsb_bits) / scale_factor;
        mag_data[1] = (float)((mag_data_binary[1]) - data_num_lsb_bits) / scale_factor;
        mag_data[2] = (float)((mag_data_binary[2]) - data_num_lsb_bits) / scale_factor;
        packet->mag_x = mag_data[0];
        packet->mag_y = mag_data[1];
        packet->mag_z = mag_data[2];
        (*flip)++; // incrememt the flip counter
        return 0;
    }
    return 1;
}

float mmc5983ma_get_magnetic_field_scale_factor(void) {
    // since the sensor has write-only config registers, this means we can't read from it
    // to determine which FS range we are at. I really don't care enough to set up shadow
    // register memory for this driver, so i'm just going to return the value we are using.
    return scale_factor;
}

int mag_setup_device(bool soft_reset_complete) {
    HAL_Delay(14); // 15ms power-on time
    uint8_t result = 0;
    // perform dummy read as required by datasheet
    HAL_StatusTypeDef hal_status = mag_i2c_read(product_id1, &result, 1);
    if (hal_status) {
        switch (hal_status) {
        case HAL_BUSY:
            serialPrintStr("\tI2C handle currently busy, unable to read");
            break;
        case HAL_ERROR:
            serialPrintStr("\tI2C read transaction failed during dummy read");
            break;
        default:
            break;
        }
        return 1;
    }
    // give device enough time to switch to correct mode
    // this is a 1ms delay
    HAL_Delay(0);

    mag_i2c_read(product_id1, &result, 1);
    if (result != product_id_val) {
        serialPrintStr("\tMagnetometer could not read Product ID");
        return 1;
    }

    // unlike the other sensors, the registers are read-only or write-only, so the startup
    // write test cannot be done, because the value of the written register cannot be verified
    // with a read

    if (soft_reset_complete) {
        // check that bit 7 (sw_rst) is back to 0
        mag_i2c_read(internal_control1, &result, 1);
        if (result & 0x80) {
            serialPrintStr("\tMMC5983MA did not complete software reset");
            return 1;
        }
    }
    return 0;

}

static HAL_StatusTypeDef mag_i2c_read(uint8_t reg_addr, uint8_t* buffer, size_t len) {
    return HAL_I2C_Mem_Read(
            I2CSettings.hi2c,
            (uint16_t)(I2CSettings.dev_addr << 1),
            (uint16_t)reg_addr,
            I2C_MEMADD_SIZE_8BIT,
            buffer,
            len,
            100);
}


static HAL_StatusTypeDef mag_i2c_write(uint8_t reg_addr, uint8_t data) {
    return HAL_I2C_Mem_Write(
            I2CSettings.hi2c,
            (uint16_t)(I2CSettings.dev_addr << 1),
            (uint16_t)reg_addr,
            I2C_MEMADD_SIZE_8BIT,
            &data,
            1,
            100);
}

