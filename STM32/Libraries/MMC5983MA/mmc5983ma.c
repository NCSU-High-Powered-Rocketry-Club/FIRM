/*
 * mmc5983ma.c
 *
 *  Created on: Sep 8, 2025
 *      Author: Wlsan
 */
#include "mmc5983ma.h"

/**
 * @brief the I2C settings for the MMC5983MA to use when accessing device registers
 */
typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint8_t dev_addr; // 7 bit i2c address for the device
} I2CSettings;

/**
 * @brief Starts up and resets the magnetometer, confirms the I2C read/write functionality is working
 *
 * @param soft_reset_complete if this is a setup after a soft reset is complete
 * @retval 0 if successful
 */
static int setup_device(bool soft_reset_complete);

/**
 * @brief Reads data from the MMC5983MA with I2C
 *
 * @param reg_addr the address of the register
 * @param buffer where the result of the read will be stored
 * @param len the number of bytes to read
 * @retval HAL Status, 0 on successful read
 */
static HAL_StatusTypeDef read_registers(uint8_t reg_addr, uint8_t* buffer, size_t len);

/**
 * @brief Writes 1 byte of data to the MMC5983MA with I2C
 *
 * @param reg_addr the address of the register
 * @param data the data to write to the register
 * @retval HAL Status, 0 on successful write
 */
static HAL_StatusTypeDef write_register(uint8_t reg_addr, uint8_t data);

// MMC5983MA register mapping
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



static I2CSettings i2cSettings;

int mmc5983ma_init(I2C_HandleTypeDef* hi2c, uint8_t device_i2c_addr) {
    if (hi2c == NULL) {
        serialPrintStr("Invalid i2c handle for MMC5983MA");
        return 1;
    }

    // configure i2c settings
    i2cSettings.hi2c = hi2c;
    i2cSettings.dev_addr = device_i2c_addr;
    serialPrintStr("Beginning MMC5983MA initialization");

    // sets up the magnetometer in I2C mode and ensures I2C is working
    if (setup_device(false)) return 1;

    // initiating a software reset
    serialPrintStr("\tIssuing MMC5983MA software reset...");
    write_register(internal_control1, 0b10000000);

    // verify correct setup again
    if (setup_device(true)) return 1;

    // enable interrupt pin
    write_register(internal_control0, 0b00000100);
    // set bandwidth to 200hz (4ms measurement time)
    write_register(internal_control1, 0b00000001);
    // enable continuous measurement mode at 200hz
    write_register(internal_control2, 0b00001110);

    serialPrintStr("\tMMC5983MA startup successful!");
    return 0;
}

int mmc5983ma_read_data(MMC5983MAPacket_t* packet, uint8_t* flip) {
    uint8_t data_ready = 0;
    // read status register to make sure data is ready
    read_registers(status, &data_ready, 1);
    if (data_ready & 0x01) { // data ready bit is 0x01
        // manually clear the interrupt signal
        write_register(status, 0b00000001);

        // every flip_interval read cycles, flip the polarity of the magnetometer values
        // to calibrate the sensor properly
        if (*flip % flip_interval == 0) {
            write_register(internal_control0, 0b00010100);
        }
        if ((*flip - 1) % flip_interval == 0) {
            write_register(internal_control0, 0b00001100);
        }
        if (*flip == 11) {
            *flip = 0;
        }

        // burst read 7 bytes of data from the first data register
        // first two bytes are magnetometer x
        // next two are magnetometer y
        // next two are magnetometer z
        // last byte is 2 bits of LSB x, 2 bits of LSB y, 2 bits of LSB z, and 2 reserved bits
        read_registers(x_out0, (uint8_t*)packet, 7);
        
        // must change from offset representation to twos complement by flipping the highest bit
        // of the msb. This effectively subtracts by half of the resolution (18 bits), centering
        // the data.
        packet->mag_x_msb ^= 0x80;
        packet->mag_y_msb ^= 0x80;
        packet->mag_z_msb ^= 0x80;
        (*flip)++; // incrememt the flip counter
        return 0;
    }
    //serialPrintStr("no data");
    return 1;
}

float mmc5983ma_get_magnetic_field_scale_factor(void) {
    // since the sensor has write-only config registers, this means we can't read from it
    // to determine which FS range we are at. I really don't care enough to set up shadow
    // register memory for this driver, so i'm just going to return the value we are using.
    return scale_factor;
}

int setup_device(bool soft_reset_complete) {
    HAL_Delay(14); // 15ms power-on time
    uint8_t result = 0;
    // perform dummy read as required by datasheet
    HAL_StatusTypeDef hal_status = read_registers(product_id1, &result, 1);
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

    read_registers(product_id1, &result, 1);
    if (result != product_id_val) {
        serialPrintStr("\tMMC5983MA could not read Product ID");
        return 1;
    }

    // unlike the other sensors, the registers are read-only or write-only, so the startup
    // write test cannot be done, because the value of the written register cannot be verified
    // with a read

    if (soft_reset_complete) {
        // check that bit 7 (sw_rst) is back to 0
        read_registers(internal_control1, &result, 1);
        if (result & 0x80) {
            serialPrintStr("\tMMC5983MA did not complete software reset");
            return 1;
        }
    }
    return 0;

}

static HAL_StatusTypeDef read_registers(uint8_t reg_addr, uint8_t* buffer, size_t len) {
    return HAL_I2C_Mem_Read(
            i2cSettings.hi2c,
            (uint16_t)(i2cSettings.dev_addr << 1),
            (uint16_t)reg_addr,
            I2C_MEMADD_SIZE_8BIT,
            buffer,
            len,
            100);
}


static HAL_StatusTypeDef write_register(uint8_t reg_addr, uint8_t data) {
    return HAL_I2C_Mem_Write(
            i2cSettings.hi2c,
            (uint16_t)(i2cSettings.dev_addr << 1),
            (uint16_t)reg_addr,
            I2C_MEMADD_SIZE_8BIT,
            &data,
            1,
            100);
}