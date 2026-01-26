/*
 * icm45686.c
 *
 *  Created on: Aug 25, 2025
 *      Author: Wlsan
 */

#include "icm45686.h"
#include "spi_utils.h"

/**
 * @brief the SPI settings for the IMU to use when accessing device registers
 */
typedef struct {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* cs_channel;
    uint16_t cs_pin;
} SPISettings;

/**
 * @brief Starts up and resets the ICM45686, confirms the SPI read/write functionality is working
 *
 * @param soft_reset_complete if this is a setup after a soft reset is complete
 * @retval 0 if successful
 */
static int setup_device(bool soft_reset_complete);

/**
 * @brief Reads data from the ICM45686 with SPI
 *
 * @param addr the address of the register
 * @param buffer where the result of the read will be stored
 * @param len the number of bytes to read
 * @retval HAL Status, 0 on successful read
 */
static HAL_StatusTypeDef read_registers(uint8_t addr, uint8_t* buffer, size_t len);

/**
 * @brief Writes 1 byte of data to the ICM45686 with SPI
 *
 * @param addr the address of the register
 * @param data the data to write to the register
 * @retval HAL Status, 0 on successful write
 */
static HAL_StatusTypeDef write_register(uint8_t addr, uint8_t data);


/**
 * @brief reads an indirect register from the ICM45686
 *
 * @note a minimum wait time of 4 microseconds is required between multiple ireg reads or writes.
 *
 * @param register_map the IREGMap enumeration value that the ireg register is located in.
 * @param ireg_addr the intended register address to read. From the datasheet, the register
 * 		  will either be a standard 8-bit register, or a 12-bit register. ireg_addr expects
 * 		  a 16-bit unsigned integer to account for this.
 * @param result a pointer to the variable that the result of the read will be stored in.
 * @retval error status, 0 on successful read
 */
static int read_ireg_register(IREGMap_t register_map, uint16_t ireg_addr, uint8_t* result);

/**
 * @brief writes to an indirect register of the ICM45686
 *
 * @note a minimum wait time of 4 microseconds is required between multiple ireg reads or writes.
 *
 * @param register_map the IREGMap enumeration value that the ireg register is located in.
 * @param ireg_addr the intended register address to write to. From the datasheet, the register
 * 		  will either be a standard 8-bit register, or a 12-bit register. ireg_addr expects
 * 		  a 16-bit unsigned integer to account for this.
 * @param data the data to write to the indirect register address, as an 8-bit unsigned integer.
 * @retval error status, 0 on successful read
 */
static int write_ireg_register(IREGMap_t register_map, uint16_t ireg_addr, uint8_t data);

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

static const float hi_res_fifo_accel_scale_factor = 16384.0F;
static const float hi_res_fifo_gyro_scale_factor = 131.072F;
static const float base_accel_scale_factor = 1024.0F;
static const float base_gyro_scale_factor = 8.192F;

static SPISettings spiSettings;

void set_spi_icm(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin) {
  spiSettings.hspi = hspi;
  spiSettings.cs_channel = cs_channel;
  spiSettings.cs_pin = cs_pin;
}

int icm45686_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin) {
    if (hspi == NULL || cs_channel == NULL) {
        serialPrintStr("Invalid spi handle or chip select pin for ICM45686");
        return 1;
    }
    // set up the SPI settings
    spiSettings.hspi = hspi;
    spiSettings.cs_channel = cs_channel;
    spiSettings.cs_pin = cs_pin;


    serialPrintStr("Beginning ICM45686 initialization");
    // sets up the IMU in SPI mode and ensures SPI is working
    if (setup_device(false)) return 1;

    // do a soft-reset of the sensor's settings
    serialPrintStr("\tIssuing ICM45686 software reset...");
    write_register(reg_misc2, 0b00000010);
    // verify correct setup again
    if (setup_device(true)) return 1;

    // sets accel range to +/- 32g, and ODR to 800hz
    write_register(accel_config0, 0b00000110);
    // sets gyro range to 4000dps, and ODR to 800hz
    write_register(gyro_config0, 0b00000110);
    // fifo set to stream mode, and 2k byte size
    write_register(fifo_config0, 0b01000111);
    // enable fifo for acceleration and gyroscope
    write_register(fifo_config3, 0b00001111);
    // disables all interrupts, to allow interrupt settings to be configured
    write_register(int1_config0, 0b00000000);
    // sets interrupt pin to push-pull, latching, and active low
    write_register(int1_config2, 0b00000010);
    // sets interrupt pin to only trigger when data is ready or reset is complete
    write_register(int1_config0, 0b10000100);

    // big endian mode
    write_ireg_register(IPREG_TOP1, (uint16_t)sreg_ctrl, 0b00000010);
    // turn interpolator and FIR filter off for gyro
    write_ireg_register(IPREG_SYS1, (uint16_t)ipreg_sys1_reg_166, 0b00001011);
    // turn interpolator and FIR filter off for acceleration
    write_ireg_register(IPREG_SYS2, (uint16_t)ipreg_sys2_reg_123, 0b00010100);
    // verify ireg read/write works
    uint8_t result = 0;
    read_ireg_register(IPREG_SYS2, (uint16_t)ipreg_sys2_reg_123, &result);
    if (result != 0b00010100) {
        serialPrintStr("\tFailed to read or write to IREG registers");
        return 1;
    }
    // place both accel and gyro in low noise mode
    write_register(pwr_mgmt0, 0b00001111);

    // delay for gyro to get ready
    HAL_Delay(74);

    // read to clear any interrupts
    read_registers(int1_status0, &result, 1);
    read_registers(int1_status1, &result, 1);
    serialPrintStr("\tICM45686 startup successful!");
    return 0;
}

int icm45686_read_data(ICM45686Packet_t* packet) {
    uint8_t data_ready = 0;
    // checking (and resetting) interrupt status
    HAL_StatusTypeDef err;
    err = read_registers(int1_status0, &data_ready, 1);
    if (data_ready & 0x04) { // bit 2 is data_ready flag for UI channel
        // each packet from the fifo is 20 bytes, with the first being the header, the next 12
        // being the most significant bytes and middle bytes of accel/gyro data, and the last 3
        // bytes of the packet are the least significant bytes of the accel/gyro data. The other
        // bytes in the packet are for timestamp and temperature data, which we discard.
        uint8_t raw_data[20];
        err = read_registers(fifo_data, raw_data, 20);
        if (err)
          return 1;
        // copying 12 bytes after the header byte (most significant byte and middle byte of accel/gyro data)
        memcpy(packet, &raw_data[1], 12);
        // copying the last 3 bytes (4-bit LSB's for accel/gyro)
        memcpy(&packet->x_vals_lsb, &raw_data[17], 3);

        // flush the fifo
        write_register(fifo_config2, 0b10100000);
        return 0;
    }
    return 1; // data was not ready, return error
}


float icm45686_get_accel_scale_factor(void) {
    if (spiSettings.hspi == NULL) {
        return -1;
    }
    uint8_t result = 0;
    read_registers(fifo_config3, &result, 1);
    if (result & 0x08) {
        // high resolution mode
        return hi_res_fifo_accel_scale_factor;
    }

    // not in fifo mode, use accel config to determine scale factor
    read_registers(accel_config0, &result, 1);
    uint8_t full_scale_selection = (result & 0x70) >> 4;
    if (full_scale_selection > 0b100) {
        return -1;
    }
    return base_accel_scale_factor * (float)pow((double)2, (double)full_scale_selection);
}

float icm45686_get_gyro_scale_factor(void) {
    if (spiSettings.hspi == NULL) {
        return -1;
    }
    uint8_t result = 0;
    read_registers(fifo_config3, &result, 1);
    if (result & 0x08) {
        // high resolution mode
        return hi_res_fifo_gyro_scale_factor;
    }

    // not in fifo mode, use accel config to determine scale factor
    read_registers(gyro_config0, &result, 1);
    uint8_t full_scale_selection = (result & 0xF0) >> 4;
    if (full_scale_selection > 0b1000) {
        return -1;
    }
    return base_gyro_scale_factor * (float)pow((double)2, (double)full_scale_selection);
}


static int setup_device(bool soft_reset_complete) {
    // datasheet says 2ms to powerup, include some factor of safety
    HAL_Delay(10);

    uint8_t result = 0;
    // perform dummy read as required by datasheet
    HAL_StatusTypeDef hal_status = read_registers(who_am_i, &result, 1);
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
    read_registers(who_am_i, &result, 1);
    if (result != 0xE9) {
        serialPrintStr("\tIMU could not read chip ID");
        return 1;
    }

    // verify that writes work by wrting to a register that has no effect on our settings
    // then set back to original value when the write succeeds.
    read_registers(fifo_config2, &result, 1);
    if (result != 0b00100000) {
        serialPrintStr("\tCould not start write test: wrong expected value for FIFO_CONFIG2");
        return 1;
    }
    write_register(fifo_config2, 0b00100100);
    read_registers(fifo_config2, &result, 1);
    if (result != 0x24) {
        serialPrintStr(
            "\tIMU SPI Write test failed, wrote to register and did not read expected value back!");
        return 1;
    }
    write_register(fifo_config2, 0b00100000); // set back to original value

    if (soft_reset_complete) {
        // Check bit 1 (soft reset bit) is set back to 0
        read_registers(reg_misc2, &result, 1);
        if ((result & 0x02) != 0) {
            serialPrintStr("\tSoftware reset failed!");
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

static int read_ireg_register(IREGMap_t register_map, uint16_t ireg_addr, uint8_t* result) {
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
        return 2;
    }
    uint8_t ireg_regs[2];
    ireg_regs[0] = (uint8_t)(ireg_addr >> 8);
    ireg_regs[1] = (uint8_t)(ireg_addr & 0x00FF);

    // Starting the burst write from IREG_ADDR_15_8 means that IREG_ADDR_7_0 will also be
    // written to, since it is auto-incremented (that's what a SPI burst write does).
    // So here, sending 2 bytes means 2 addresses (IREG_ADDR_15_8 and IREG_ADDR_7_0) are
    // configured at once.
    int error_status = spi_burst_write(spiSettings.hspi, spiSettings.cs_channel, spiSettings.cs_pin, ireg_addr_15_8, ireg_regs, sizeof(ireg_regs));

    // After the write is over and the CS pin is pulled high, result of the the read
    // will be stored in the IREG_DATA register. The `result` pointer will have the value:
    // NOTE: We must wait for a minimum of 4us before reading IREG_DATA:

    HAL_Delay(0); // can only do millisecond delays, so 1ms is enough
    if (error_status) return error_status;
    error_status = read_registers(ireg_data, result, 1);

    return error_status;
}

static int write_ireg_register(IREGMap_t register_map, uint16_t ireg_addr, uint8_t data) {
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
        return 2;
    }

    // save the page, register, and data into an array
    uint8_t ireg_regs[3];
    ireg_regs[0] = (uint8_t)(ireg_addr >> 8);
    ireg_regs[1] = (uint8_t)(ireg_addr & 0x00FF);
    ireg_regs[2] = data;

    // burst write page,register, and data starting at ireg_addr_15_8
    int error_status = spi_burst_write(spiSettings.hspi, spiSettings.cs_channel, spiSettings.cs_pin, ireg_addr_15_8, ireg_regs, 3);
    // must wait before next ireg operation
    // this is a 1ms delay
    HAL_Delay(0);
    return error_status;
}
