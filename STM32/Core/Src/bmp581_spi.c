/*
 * bmp581_spi.c
 *
 *  Created on: Aug 21, 2025
 *      Author: Wlsan
 */
#include "bmp581_spi.h"
#include "packets.h"
#include "spi_utils.h"
#include <stdint.h>

// register for chip product ID (to test SPI transfers)
const uint8_t bmp581_reg_chip_id = 0x01;
const uint8_t bmp581_reg_asic_rev_id = 0x02;
const uint8_t bmp581_reg_chip_status = 0x11;
const uint8_t bmp581_reg_int_config = 0x14;
const uint8_t bmp581_reg_int_source = 0x15;
const uint8_t bmp581_reg_fifo_sel = 0x18;
const uint8_t bmp581_reg_temp_data_xlsb = 0x1D;
const uint8_t bmp581_reg_int_status = 0x27;
const uint8_t bmp581_reg_status = 0x28;
const uint8_t bmp581_reg_osr_config = 0x36;
const uint8_t bmp581_reg_ord_config = 0x37;
const uint8_t bmp581_reg_cmd = 0x7E;

// BMP SPI config settings
static BMPSPISettings SPISettings;

int bmp_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin) {
    serialPrintStr("Beginning BMP581 initialization");
    // set up the SPI settings
    SPISettings.hspi = hspi;
    SPISettings.cs_channel = cs_channel;
    SPISettings.cs_pin = cs_pin;
    // sets up the BMP in SPI mode and ensures SPI is working
    if (bmp_setup_device(false)) {
        return 1;
    }
    serialPrintStr("Issuing BMP581 software reset...");
    bmp_spi_write(bmp581_reg_cmd, 0b10110110); // do a soft-reset of the sensor's settings
    if (bmp_setup_device(true)) {              // verify correct setup again
        return 1;
    }

    // enable pressure measurements, sets 1x over-sampling (no OSR) for pressure and temperature.
    spi_write(hspi, cs_channel, cs_pin, bmp581_reg_osr_config, 0b01000000);
    // enable interrupt pin, set to active-low, open-drain, latched mode
    spi_write(hspi, cs_channel, cs_pin, bmp581_reg_int_config, 0b00111001);
    // set the source of the interrupt signal to be on data-ready
    spi_write(hspi, cs_channel, cs_pin, bmp581_reg_int_source, 0b00000001);
    // disable deep-sleep, set to max ODR, set to continuous mode
    spi_write(hspi, cs_channel, cs_pin, bmp581_reg_ord_config, 0b10000011);
    // continuous mode actually ignores the ODR bits that were set, and uses the OSR to determine
    // the ODR (498hz with 1x OSR)

    return 0;
}

int bmp_read(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin,
             BMPPacket_t* packet) {
    // clear interrupt (pulls interrupt back up high) and verify new data is ready
    uint8_t data_ready = 0;
    spi_read(hspi, cs_channel, cs_pin, bmp581_reg_int_status, &data_ready, 1);
    if (data_ready & 0x01) { // bit 0 (LSB) will be 1 if new data is ready
        // temperature and pressure are both 24 bit values, with the data in 3 registers each
        // burst read 6 registers starting from XLSB of temp, to MSB of pressure (0x1D -> 0x22)
        uint8_t raw_data[6];
        spi_read(hspi, cs_channel, cs_pin, bmp581_reg_temp_data_xlsb, raw_data, 6);
        // bit shift the raw data, MSB shifts 16 bits left, LSB 8 bits left, and XLSB rightmost
        int32_t raw_temp = ((int32_t)raw_data[2] << 16) | ((int32_t)raw_data[1] << 8) | raw_data[0];
        uint32_t raw_pres =
            ((uint32_t)raw_data[5] << 16) | ((uint32_t)raw_data[4] << 8) | raw_data[3];
        // datasheet instructs to divide raw temperature by 2^16 to get value in celcius, and
        // divide raw pressure by 2^6 to get value in Pascals
        packet->temperature = raw_temp / 65536.0f;
        packet->pressure = raw_pres / 64.0f;
        // serialPrintFloat(temp);
        // serialPrintFloat(pres);
        return 0;
    }
    return 1;
}

int bmp_setup_device(bool soft_reset_complete) {
    // datasheet says 2ms to powerup, include some factor of safety
    HAL_Delay(10);

    uint8_t result = 0;
    // perform dummy read as required by datasheet
    HAL_StatusTypeDef hal_status = bmp_spi_read(bmp581_reg_chip_id, &result, 1);
    if (hal_status) {
        switch (hal_status) {
        case HAL_BUSY:
            serialPrintStr("SPI handle currently busy, unable to read");
            break;
        case HAL_TIMEOUT:
            serialPrintStr("SPI read timed out during dummy read");
            break;
        case HAL_ERROR:
            serialPrintStr("SPI read transaction failed during dummy read");
            break;
        default:
            break;
        }
        return 1;
    }
    HAL_Delay(1);

    // ensure that device is set up in SPI Mode0/Mode3, and no errors are listed
    bmp_spi_read(bmp581_reg_chip_status, &result, 1);
    if (result != 0x02) {
        if (!(result & 0x03)) {
            serialPrintStr("BMP wrongly initialized in I2C mode");
        } else if ((result & 0x03) == 0x03) {
            serialPrintStr("BMP communication mode is in autoconfig mode. Check pull-up resistors");
        } else if ((result & 0x03) == 0x01) {
            serialPrintStr("BMP wrongly initialized in SPI Mode 1/2");
        } else {
            serialPrintStr("I3C error");
        }
        return 1;
    }

    // verify chip ID and asic rev ID read works
    bmp_spi_read(bmp581_reg_chip_id, &result, 1);
    if (result != 0x50) {
        serialPrintStr("BMP could not read chip ID");
        return 1;
    }
    bmp_spi_read(bmp581_reg_asic_rev_id, &result, 1);
    if (result != 0x32) {
        serialPrintStr("BMP could not read ASIC revision ID");
        return 1;
    }

    // verify that writes work
    bmp_spi_read(bmp581_reg_fifo_sel, &result, 1);
    if (result) {
        serialPrintStr("Could not start write test: wrong expected value for FIFO_SEL");
        return 1;
    }
    bmp_spi_write(bmp581_reg_fifo_sel, 0b00000100);
    bmp_spi_read(bmp581_reg_fifo_sel, &result, 1);
    if (result != 0x04) {
        serialPrintStr(
            "BMP SPI Write test failed, wrote to register and did not read expected value back!");
        return 1;
    }
    bmp_spi_write(bmp581_reg_fifo_sel, 0b00000000); // set back to default

    // verify device is ready to be configured
    bmp_spi_read(bmp581_reg_status, &result, 1);
    if (result != 0x02) {
        if (result & 0x04) {
            serialPrintStr("NVM error, refer to datasheet for source of error");
        }
        if (result & 0x08) {
            serialPrintStr("NVM error, datasheet just says \"TODO UPDATE ME\"");
        }
        return 1;
    }

    if (soft_reset_complete) {
        // verify software reset is recognized as complete by the interrupt status register
        bmp_spi_read(bmp581_reg_int_status, &result, 1);
        if (!(result & 0x10)) { // check that bit 4 (POR) is 1
            serialPrintStr("Software reset interrupt signal not generated!");
            return 1;
        }
    }
    serialPrintStr("BMP startup successful");
    return 0;
}

HAL_StatusTypeDef bmp_spi_read(uint8_t addr, uint8_t* buffer, size_t len) {
    return spi_read(SPISettings.hspi, SPISettings.cs_channel, SPISettings.cs_pin, addr, buffer,
                    len);
}

HAL_StatusTypeDef bmp_spi_write(uint8_t addr, uint8_t data) {
    return spi_write(SPISettings.hspi, SPISettings.cs_channel, SPISettings.cs_pin, addr, data);
}
