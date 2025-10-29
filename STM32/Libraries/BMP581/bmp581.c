/*
 * bmp581.c
 *
 *  Created on: Aug 21, 2025
 *      Author: Wlsan
 */
#include "bmp581.h"
#include <spi_utils.h>
#include <stdint.h>


/**
 * @brief the SPI settings for the BMP581 to use when accessing device registers
 */
typedef struct {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* cs_channel;
    uint16_t cs_pin;
} SPISettings;

/**
 * @brief Starts up and resets the BMP581, confirms the SPI read/write functionality is working
 *
 * @param soft_reset_complete if this is a setup after a soft reset is complete
 * @retval 0 if successful
 */
static int bmp581_setup_device(bool soft_reset_complete);

/**
 * @brief Reads data from the BMP581 with SPI
 *
 * @param addr the address of the register
 * @param buffer where the result of the read will be stored
 * @param len the number of bytes to read
 * @retval HAL Status, 0 on successful read
 */
static HAL_StatusTypeDef read_registers(uint8_t addr, uint8_t* buffer, size_t len);

/**
 * @brief Writes 1 byte of data to the BMP581 with SPI
 *
 * @param addr the address of the register
 * @param data the data to write to the register
 * @retval HAL Status, 0 on successful write
 */
static HAL_StatusTypeDef write_register(uint8_t addr, uint8_t data);

// register for chip product ID (to test SPI transfers)
static const uint8_t chip_id = 0x01;
static const uint8_t asic_rev_id = 0x02;
static const uint8_t chip_status = 0x11;
static const uint8_t int_config = 0x14;
static const uint8_t int_source = 0x15;
static const uint8_t fifo_sel = 0x18;
static const uint8_t temp_data_xlsb = 0x1D;
static const uint8_t int_status = 0x27;
static const uint8_t status = 0x28;
static const uint8_t osr_config = 0x36;
static const uint8_t ord_config = 0x37;
static const uint8_t cmd = 0x7E;

// device scale factor
static const int scale_factor_celcius = 65536.0F;
static const int scale_factor_pascal = 64.0F;

// BMP581 SPI config settings
static SPISettings spiSettings;

int bmp581_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin) {
    if (hspi == NULL || cs_channel == NULL) {
        serialPrintStr("Invalid spi handle or chip select pin for BMP581");
        return 1;
    }
    // set up the SPI settings
    spiSettings.hspi = hspi;
    spiSettings.cs_channel = cs_channel;
    spiSettings.cs_pin = cs_pin;


    serialPrintStr("Beginning BMP581 initialization");
    // sets up the BMP581 in SPI mode and ensures SPI is working
    if (bmp581_setup_device(false)) {
        return 1;
    }
    serialPrintStr("\tIssuing BMP581 software reset...");
    write_register(cmd, 0b10110110); // do a soft-reset of the sensor's settings
    if (bmp581_setup_device(true)) {              // verify correct setup again
        return 1;
    }

    // enable pressure measurements, sets 1x over-sampling (no OSR) for pressure and temperature.
    write_register(osr_config, 0b01000000);
    // enable interrupt pin, set to active-low, open-drain, latched mode
    write_register(int_config, 0b00111001);
    // set the source of the interrupt signal to be on data-ready
    write_register(int_source, 0b00000001);
    // disable deep-sleep, set to max ODR, set to continuous mode
    write_register(ord_config, 0b10000011);
    // continuous mode actually ignores the ODR bits that were set, and uses the OSR to determine
    // the ODR (498hz with 1x OSR)
    serialPrintStr("\tBMP581 startup successful!");
    return 0;
}

int bmp581_read_data(BMP581Packet_t* packet) {
    // clear interrupt (pulls interrupt back up high) and verify new data is ready
    uint8_t data_ready = 0;
    read_registers(int_status, &data_ready, 1);
    if (data_ready & 0x01) { // bit 0 (LSB) will be 1 if new data is ready
        // temperature and pressure are both 24 bit values, with the data in 3 registers each
        // burst read 6 registers starting from XLSB of temp, to MSB of pressure (0x1D -> 0x22).
        // the packet is passed in, so the retrieved values get stored directly into the desired
        // place in memory.
        read_registers(temp_data_xlsb, (uint8_t*)packet, 6);
        return 0;
    }
    return 1;
}


float bmp581_get_temp_scale_factor(void) {
    return scale_factor_celcius;
}


float bmp581_get_pressure_scale_factor(void) {
    return scale_factor_pascal;
}

static int bmp581_setup_device(bool soft_reset_complete) {
    // datasheet says 2ms to powerup, include some factor of safety
    HAL_Delay(10);

    uint8_t result = 0;
    // perform dummy read as required by datasheet
    HAL_StatusTypeDef hal_status = read_registers(chip_id, &result, 1);
    if (hal_status) {
        switch (hal_status) {
        case HAL_BUSY:
            serialPrintStr("\tSPI handle currently busy, unable to read");
            break;
        case HAL_TIMEOUT:
            serialPrintStr("\tSPI read timed out during dummy read");
            break;
        case HAL_ERROR:
            serialPrintStr("\tSPI read transaction failed during dummy read");
            break;
        default:
            break;
        }
        return 1;
    }
    // give device enough time to switch to correct mode
    HAL_Delay(0);

    // ensure that device is set up in SPI Mode0/Mode3, or autoconfig mode
    read_registers(chip_status, &result, 1);
    if (result == 0x00) {
        serialPrintStr("\tBMP581 wrongly initialized in I2C mode");
        uint8_t read_check;
        read_registers(chip_id, &read_check, 1);
        if (read_check != 0x50) {
            serialPrintStr("\tBMP581 chip ID read failed, device is most likely not wired correctly");
        }
        
        return 1;
    }
    if (result == 0x01) {
        serialPrintStr("\tBMP581 wrongly initialized in SPI Mode 1/2");
        return 1;
    }
    if (result & 0x0C) {
        serialPrintStr("\tI3C error, check datasheet register 0x11 for more info");
        return 1;
    }

    // verify chip ID and asic rev ID read works
    read_registers(chip_id, &result, 1);
    if (result != 0x50) {
        serialPrintStr("\tBMP581 could not read chip ID");
        return 1;
    }
    read_registers(asic_rev_id, &result, 1);
    if (result != 0x32) {
        serialPrintStr("\tBMP581 could not read ASIC revision ID");
        return 1;
    }

    // verify that writes work
    read_registers(fifo_sel, &result, 1);
    if (result) {
        serialPrintStr("\tCould not start write test: wrong expected value for FIFO_SEL");
        return 1;
    }
    write_register(fifo_sel, 0b00000100);
    read_registers(fifo_sel, &result, 1);
    if (result != 0x04) {
        serialPrintStr(
            "\tBMP581 SPI Write test failed, wrote to register and did not read expected value back!");
        return 1;
    }
    write_register(fifo_sel, 0b00000000); // set back to default

    // verify device is ready to be configured
    read_registers(status, &result, 1);
    if (result & 0x04) {
        serialPrintStr("\tNVM error, refer to datasheet for source of error");
        return 1;
    }
    if (result & 0x08) {
        serialPrintStr("\tNVM command error, refer to datasheet for source of error");
        return 1;
    }
    
    if (soft_reset_complete) {
        

        // verify software reset is recognized as complete by the interrupt status register
        read_registers(int_status, &result, 1);
        if (!(result & 0x10)) { // check that bit 4 (POR) is 1
            serialPrintStr("\tSoftware reset interrupt signal not generated!");
            return 1;
        }
    }
    return 0;
}

static HAL_StatusTypeDef read_registers(uint8_t addr, uint8_t* buffer, size_t len) {
    return spi_read(spiSettings.hspi, spiSettings.cs_channel, spiSettings.cs_pin, addr, buffer,
                    len);
}

static HAL_StatusTypeDef write_register(uint8_t addr, uint8_t data) {
    return spi_write(spiSettings.hspi, spiSettings.cs_channel, spiSettings.cs_pin, addr, data);
}
