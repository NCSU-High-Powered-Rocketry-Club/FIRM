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
const float pi = 3.14159265F;

static const uint8_t pwr_mgmt0 = 0x10;
static const uint8_t fifo_data = 0x14;
static const uint8_t int1_config0 = 0x16;
static const uint8_t int1_config2 = 0x18;
static const uint8_t int1_status0 = 0x19;
static const uint8_t int1_status1 = 0x1A;
static const uint8_t accel_config0 = 0x1B;
static const uint8_t gyro_config0 = 0x1C;
static const uint8_t fifo_config0 = 0x1D;
static const uint8_t fifo_config2 = 0x20;
static const uint8_t fifo_config3 = 0x21;
static const uint8_t who_am_i = 0x72;
static const uint8_t ireg_addr_15_8 = 0x7C; // used to read/write from indirect registers (IREG)
static const uint8_t ireg_data = 0x7E;
static const uint8_t reg_misc2 = 0x7F;
static const uint8_t sreg_ctrl = 0x67;          // IPREG_TOP1 register
static const uint8_t ipreg_sys1_reg_166 = 0xA6; // IPREG_SYS1 register
static const uint8_t ipreg_sys2_reg_123 = 0x7B; // IPREG_SYS2 register

static IMUSPISettings SPISettings;

int imu_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin) {
    if (hspi == NULL || cs_channel == NULL) {
        serialPrintStr("Invalid spi handle or chip select pin for ICM45686");
        return 1;
    }
    // set up the SPI settings
    SPISettings.hspi = hspi;
    SPISettings.cs_channel = cs_channel;
    SPISettings.cs_pin = cs_pin;

    serialPrintStr("Beginning ICM45686 initialization");
    // sets up the IMU in SPI mode and ensures SPI is working
    if (imu_setup_device(false))
        return 1;

    // do a soft-reset of the sensor's settings
    serialPrintStr("\tIssuing ICM45686 software reset...");
    imu_spi_write(reg_misc2, 0b00000010);
    // verify correct setup again
    if (imu_setup_device(true))
        return 1;

    // sets accel range to +/- 32g, and ODR to 800hz
    imu_spi_write(accel_config0, 0b00000110);
    // sets gyro range to 4000dps, and ODR to 800hz
    imu_spi_write(gyro_config0, 0b00000110);
    // fifo set to stream mode, and 2k byte size
    imu_spi_write(fifo_config0, 0b01000111);
    // enable fifo for acceleration and gyroscope
    imu_spi_write(fifo_config3, 0b00001111);
    // disables all interrupts, to allow interrupt settings to be configured
    imu_spi_write(int1_config0, 0b00000000);
    // sets interrupt pin to push-pull, latching, and active low
    imu_spi_write(int1_config2, 0b00000010);
    // sets interrupt pin to only trigger when data is ready or reset is complete
    imu_spi_write(int1_config0, 0b10000100);

    // big endian mode
    spi_ireg_write(IPREG_TOP1, (uint16_t)sreg_ctrl, 0b00000010);
    // turn interpolator and FIR filter off for gyro
    spi_ireg_write(IPREG_SYS1, (uint16_t)ipreg_sys1_reg_166, 0b00001011);
    // turn interpolator and FIR filter off for acceleration
    spi_ireg_write(IPREG_SYS2, (uint16_t)ipreg_sys2_reg_123, 0b00010100);
    // verify ireg read/write works
    uint8_t result = 0;
    spi_ireg_read(IPREG_SYS2, (uint16_t)ipreg_sys2_reg_123, &result);
    if (result != 0b00010100) {
        serialPrintStr("\tFailed to read or write to IREG registers");
        return 1;
    }
    // place both accel and gyro in low noise mode
    imu_spi_write(pwr_mgmt0, 0b00001111);

    // delay for gyro to get ready
    HAL_Delay(74);

    // read to clear any interrupts
    imu_spi_read(int1_status0, &result, 1);
    imu_spi_read(int1_status1, &result, 1);
    serialPrintStr("\tICM45686 startup successful!");
    return 0;
}

int imu_read_data(IMUPacket_t* packet) {
    uint8_t data_ready = 0;
    // checking (and resetting) interrupt status
    imu_spi_read(int1_status0, &data_ready, 1);
    if (data_ready & 0x04) { // bit 2 is data_ready flag for UI channel
        // each packet from the fifo is 20 bytes
        uint8_t raw_data[20];
        imu_spi_read(fifo_data, raw_data, 20);

        // refer to the datasheet section 6.1 "packet structure" for information on the packet
        // structure to see which bytes of the FIFO packet go to which data points.
        // Accel X
        uint32_t temp =
            ((uint32_t)raw_data[1] << 12) | ((uint32_t)raw_data[2] << 4) | (raw_data[17] >> 4);
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
        temp =
            ((uint32_t)raw_data[9] << 12) | ((uint32_t)raw_data[10] << 4) | (raw_data[18] & 0x0F);
        int32_t gy = sign_extend_20bit(temp);

        // Gyro Z
        temp =
            ((uint32_t)raw_data[11] << 12) | ((uint32_t)raw_data[12] << 4) | (raw_data[19] & 0x0F);
        int32_t gz = sign_extend_20bit(temp);

        // TODO: determine whether the data should be logged before or after the scale factor
        // is applied.

        // datasheet lists the scale factor for accelerometer to be 16,384 LSB/g when in FIFO mode
        packet->acc_x = (float)ax / 16384.0F;
        packet->acc_y = (float)ay / 16384.0F;
        packet->acc_z = (float)az / 16384.0F;
        // datasheet lists gyroscope scale factor as 131.072 LSB/(deg/s). We will also convert to
        // radians, coming out to 23592.96 / PI
        packet->gyro_x = (float)gx / (23592.96F / pi);
        packet->gyro_y = (float)gy / (23592.96F / pi);
        packet->gyro_z = (float)gz / (23592.96F / pi);

        // flush the fifo
        imu_spi_write(fifo_config2, 0b10100000);
        return 0;
    }
    return 1; // data was not ready, return error
}

int imu_setup_device(bool soft_reset_complete) {
    // datasheet says 2ms to powerup, include some factor of safety
    HAL_Delay(10);

    uint8_t result = 0;
    // perform dummy read as required by datasheet
    HAL_StatusTypeDef hal_status = imu_spi_read(who_am_i, &result, 1);
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
    // this is a 1ms delay
    HAL_Delay(0);

    // verify chip ID read works
    imu_spi_read(who_am_i, &result, 1);
    if (result != 0xE9) {
        serialPrintStr("\tIMU could not read chip ID");
        return 1;
    }

    // verify that writes work by wrting to a register that has no effect on our settings
    // then set back to original value when the write succeeds.
    imu_spi_read(fifo_config2, &result, 1);
    if (result != 0b00100000) {
        serialPrintStr("\tCould not start write test: wrong expected value for FIFO_CONFIG2");
        return 1;
    }
    imu_spi_write(fifo_config2, 0b00100100);
    imu_spi_read(fifo_config2, &result, 1);
    if (result != 0x24) {
        serialPrintStr(
            "\tIMU SPI Write test failed, wrote to register and did not read expected value back!");
        return 1;
    }
    imu_spi_write(fifo_config2, 0b00100000); // set back to original value

    if (soft_reset_complete) {
        // Check bit 1 (soft reset bit) is set back to 0
        imu_spi_read(reg_misc2, &result, 1);
        if ((result & 0x02) != 0) {
            serialPrintStr("\tSoftware reset failed!");
            return 1;
        }
    }
    return 0;
}

HAL_StatusTypeDef imu_spi_read(uint8_t addr, uint8_t* buffer, size_t len) {
    return spi_read(SPISettings.hspi, SPISettings.cs_channel, SPISettings.cs_pin, addr, buffer,
                    len);
}

HAL_StatusTypeDef imu_spi_write(uint8_t addr, uint8_t data) {
    return spi_write(SPISettings.hspi, SPISettings.cs_channel, SPISettings.cs_pin, addr, data);
}

void spi_ireg_read(IREGMap_t register_map, uint16_t ireg_addr, uint8_t* result) {
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
    spi_burst_write(SPISettings.hspi, SPISettings.cs_channel, SPISettings.cs_pin, ireg_addr_15_8,
                    ireg_regs, sizeof(ireg_regs));

    // After the write is over and the CS pin is pulled high, result of the the read
    // will be stored in the IREG_DATA register. The `result` pointer will have the value:
    // NOTE: We must wait for a minimum of 4us before reading IREG_DATA:

    HAL_Delay(0); // can only do millisecond delays, so 1ms is enough

    imu_spi_read(ireg_data, result, 1);
}

void spi_ireg_write(IREGMap_t register_map, uint16_t ireg_addr, uint8_t data) {
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
    spi_burst_write(SPISettings.hspi, SPISettings.cs_channel, SPISettings.cs_pin, ireg_addr_15_8,
                    ireg_regs, 3);
    // must wait before next ireg operation
    // this is a 1ms delay
    HAL_Delay(0);
}
