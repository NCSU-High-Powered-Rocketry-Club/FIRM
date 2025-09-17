/*
 * icm45686.c
 *
 *  Created on: Aug 25, 2025
 *      Author: Wlsan
 */

#include "icm45686.h"
#include "firm_utils.h"
#include "packets.h"
#include "spi_utils.h"

// honestly i think this is probably a good thing do make a preprocessor macro but probably later.
// most likely don't need more digits because we only have a single-precision FPU
const float pi = 3.14159265;

const uint8_t accel_data_x_ui_msb = 0x00;
const uint8_t pwr_mgmt0 = 0x10;
const uint8_t fifo_data = 0x14;
const uint8_t int1_config0 = 0x16;
const uint8_t int1_config2 = 0x18;
const uint8_t int1_status0 = 0x19;
const uint8_t int1_status1 = 0x1A;
const uint8_t accel_config0 = 0x1B;
const uint8_t gyro_config0 = 0x1C;
const uint8_t fifo_config0 = 0x1D;
const uint8_t fifo_config2 = 0x20;
const uint8_t fifo_config3 = 0x21;
const uint8_t who_am_i = 0x72;
const uint8_t ireg_addr_15_8 = 0x7C; // used to read/write from indirect registers (IREG)
const uint8_t ireg_addr_7_0 = 0x7D;  // used to read/write from indirect registers (IREG)
const uint8_t ireg_data = 0x7E;
const uint8_t reg_misc2 = 0x7F;
const uint8_t sreg_ctrl = 0x67;          // IPREG_TOP1 register
const uint8_t ipreg_sys1_reg_166 = 0xA6; // IPREG_SYS1 register
const uint8_t ipreg_sys2_reg_123 = 0x7B; // IPREG_SYS2 register

int imu_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin) {

    HAL_Delay(2); // 3ms delay to allow device to power on

    // Do a dummy read
    uint8_t result = 0;
    spi_read(hspi, cs_channel, cs_pin, who_am_i, &result, 1);
    result = 0;
    // Verify chip ID is 0xE9
    spi_read(hspi, cs_channel, cs_pin, who_am_i, &result, 1);
    if (result != 0xE9) {
        serialPrintStr("icm45686 chip ID failed to read");
        return 1;
    }

    // issues a soft reset
    spi_write(hspi, cs_channel, cs_pin, reg_misc2, 0b00000010);
    HAL_Delay(2); // 3ms delay to allow device to finish reset
    result = 0;

    // Do another dummy read
    spi_read(hspi, cs_channel, cs_pin, who_am_i, &result, 1);
    // Verify chip ID is 0xE9
    spi_read(hspi, cs_channel, cs_pin, who_am_i, &result, 1);
    if (result != 0xE9) {
        serialPrintStr("icm45686 chip ID failed to read");
        return 1;
    }

    // Check bit 1 (soft reset bit) is set back to 0
    spi_read(hspi, cs_channel, cs_pin, reg_misc2, &result, 1);
    if ((result & 0x02) != 0) {
        serialPrintStr("icm45686 software reset failed");
        return 1;
    }

    // sets accel range to +/- 32g, and ODR to 800hz
    spi_write(hspi, cs_channel, cs_pin, accel_config0, 0b00000110);
    // sets gyro range to 4000dps, and ODR to 800hz
    spi_write(hspi, cs_channel, cs_pin, gyro_config0, 0b00000110);
    // fifo set to stream mode, and 2k byte size
    spi_write(hspi, cs_channel, cs_pin, fifo_config0, 0b01000111);
    // enable fifo for acceleration and gyroscope
    spi_write(hspi, cs_channel, cs_pin, fifo_config3, 0b00001111);
    // disables all interrupts, to allow interrupt settings to be configured
    spi_write(hspi, cs_channel, cs_pin, int1_config0, 0b00000000);
    // sets interrupt pin to push-pull, latching, and active low
    spi_write(hspi, cs_channel, cs_pin, int1_config2, 0b00000010);
    // sets interrupt pin to only trigger when data is ready or reset is complete
    spi_write(hspi, cs_channel, cs_pin, int1_config0, 0b10000100);

    // big endian mode
    spi_ireg_write(hspi, cs_channel, cs_pin, IPREG_TOP1, (uint16_t)sreg_ctrl, 0b00000010);
    // turn interpolator and FIR filter off for gyro
    spi_ireg_write(hspi, cs_channel, cs_pin, IPREG_SYS1, (uint16_t)ipreg_sys1_reg_166, 0b00001011);
    // turn interpolator and FIR filter off for acceleration
    spi_ireg_write(hspi, cs_channel, cs_pin, IPREG_SYS2, (uint16_t)ipreg_sys2_reg_123, 0b00010100);
    spi_ireg_read(hspi, cs_channel, cs_pin, IPREG_SYS2, (uint16_t)ipreg_sys2_reg_123, &result);
    if (result != 0b00010100) {
        serialPrintStr("icm45686 failed to write");
        return 1;
    }
    // place both accel and gyro in low noise mode
    spi_write(hspi, cs_channel, cs_pin, pwr_mgmt0, 0b00001111);
    // read to clear any interrupts
    spi_read(hspi, cs_channel, cs_pin, int1_status0, &result, 1);
    spi_read(hspi, cs_channel, cs_pin, int1_status1, &result, 1);
    // delay for gyro to get ready
    HAL_Delay(74);
    spi_read(hspi, cs_channel, cs_pin, int1_status0, &result, 1);
    serialPrintStr("ICM45686 setup complete");
    return 0;
}



int imu_read(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin,
             IMUPacket_t* packet) {
    uint8_t data_ready = 0;
    // checking (and resetting) interrupt status
    spi_read(hspi, cs_channel, cs_pin, int1_status0, &data_ready, 1);
    if (data_ready & 0x04) { // bit 2 is data_ready flag for UI channel
        // each packet from the fifo is 20 bytes
        uint8_t raw_data[20];
        spi_read(hspi, cs_channel, cs_pin, fifo_data, raw_data, 20);

        // refer to the datasheet section 6.1 "packet structure" for how the bytes of the
        // FIFO packet are arranged. We ignore bytes 13 - 16 (temp and timestamp)
        // Accel X
        uint32_t temp = ((uint32_t)raw_data[1] << 12) | ((uint32_t)raw_data[2] << 4) | (raw_data[17] >> 4);
        int32_t ax = sign_extend_20bit(temp);
        // Accel Y
        temp = ((uint32_t)raw_data[3] << 12) | ((uint32_t)raw_data[4] << 4) | (raw_data[18] >> 4);
        int32_t ay = sign_extend_20bit(temp);

        // Accel Z
        temp = ((uint32_t)raw_data[5] << 12) | ((uint32_t)raw_data[6] << 4) | (raw_data[19] >> 4);
        int32_t az = sign_extend_20bit(temp);

        // Gyro X
        temp = ((uint32_t)raw_data[7] << 12) | ((uint32_t)raw_data[8] << 4) | (raw_data[17] & 0x0F);
        int32_t gx = sign_extend_20bit(temp);

        // Gyro Y
        temp = ((uint32_t)raw_data[9] << 12) | ((uint32_t)raw_data[10] << 4) | (raw_data[18] & 0x0F);
        int32_t gy = sign_extend_20bit(temp);

        // Gyro Z
        temp = ((uint32_t)raw_data[11] << 12) | ((uint32_t)raw_data[12] << 4) | (raw_data[19] & 0x0F);
        int32_t gz = sign_extend_20bit(temp);

        // TODO: determine whether the data should be logged before or after the scale factor
        // is applied.

        // datasheet lists the scale factor for accelerometer to be 16,384 LSB/g when in FIFO mode
        packet->acc_x = (float)ax / 16384.0f;
        packet->acc_y = (float)ay / 16384.0f;
        packet->acc_z = (float)az / 16384.0f;
        // datasheet lists gyroscope scale factor as 131.072 LSB/(deg/s). We will also convert to
        // radians, coming out to 23592.96 / PI
        packet->gyro_x = (float)gx / (23592.96f / pi);
        packet->gyro_y = (float)gy / (23592.96f / pi);
        packet->gyro_z = (float)gz / (23592.96f / pi);

        // flush the fifo
        spi_write(hspi, cs_channel, cs_pin, fifo_config2, 0b10100000);
        return 0;
    }
    return 1; // data was not ready, return error
}

void spi_ireg_read(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin,
                   IREGMap_t register_map, uint16_t ireg_addr, uint8_t* result) {
    switch (register_map) {
    case IMEM_SRAM:
        ireg_addr |= 0x0000;
        break;
    case IPREG_BAR:
        ireg_addr |= 0xA000;
        break;
    case IPREG_SYS1:
        ireg_addr |= 0xA400;
        break;
    case IPREG_SYS2:
        ireg_addr |= 0xA500;
        break;
    case IPREG_TOP1:
        ireg_addr |= 0xA200;
        break;
    default:
        return;
    }
    uint8_t ireg_regs[2];
    ireg_regs[0] = (uint8_t)(ireg_addr >> 8);
    ireg_regs[1] = (uint8_t)(ireg_addr & 0x00FF);

    // Starting the burst write from IREG_ADDR_15_8 means that IREG_ADDR_7_0 will also be
    // written to, since it is auto-incremented (that's what a SPI burst write does).
    // So here, sending 2 bytes means 2 addresses (IREG_ADDR_15_8 and IREG_ADDR_7_0) are
    // configured at once.
    spi_burst_write(hspi, cs_channel, cs_pin, ireg_addr_15_8, ireg_regs, sizeof(ireg_regs));

    // After the write is over and the CS pin is pulled high, result of the the read
    // will be stored in the IREG_DATA register. The `result` pointer will have the value:
    // NOTE: We must wait for a minimum of 4us before reading IREG_DATA:

    HAL_Delay(0); // can only do millisecond delays, so 1ms is enough

    spi_read(hspi, cs_channel, cs_pin, ireg_data, result, 1);
}

void spi_ireg_write(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin,
                    IREGMap_t register_map, uint16_t ireg_addr, uint8_t data) {
    // combine page with address in as 16 bits
    switch (register_map) {
    case IMEM_SRAM:
        ireg_addr |= 0x0000;
        break;
    case IPREG_BAR:
        ireg_addr |= 0xA000;
        break;
    case IPREG_SYS1:
        ireg_addr |= 0xA400;
        break;
    case IPREG_SYS2:
        ireg_addr |= 0xA500;
        break;
    case IPREG_TOP1:
        ireg_addr |= 0xA200;
        break;
    default:
        return;
    }

    // save the page, register, and data into an array
    uint8_t ireg_regs[3];
    ireg_regs[0] = (uint8_t)(ireg_addr >> 8);
    ireg_regs[1] = (uint8_t)(ireg_addr & 0x00FF);
    ireg_regs[2] = data;

    // burst write page,register, and data starting at ireg_addr_15_8
    spi_burst_write(hspi, cs_channel, cs_pin, ireg_addr_15_8, ireg_regs, 3);
    // must wait before next ireg operation
    HAL_Delay(0);
}

