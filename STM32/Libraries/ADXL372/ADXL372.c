#include "ADXL372.h"
#include "spi_utils.h"

/**
 * @brief the SPI settings for the ADXL372 to use when accessing device registers
 */

typedef struct{
    SPI_TypeDef* hspi;
    GPIO_TypeDef* cs_channel;
    unit16_t cs_pin;
}SPISettings;

/**
 * @brief Starts up and resets the ADXL372, confirms the SPI read/write functionality is working
 *
 * @param soft_reset_complete if this is a setup after a soft reset is complete
 * @retval 0 if successful
 */
static int setup_device(bool soft_reset_complete);

/**
 * @brief Reads data from the ADXL372 with SPI
 *
 * @param addr the address of the register
 * @param buffer where the result of the read will be stored
 * @param len the number of bytes to read
 * @retval HAL Status, 0 on successful read
 */
static HAL_StatusTypeDef read_registers(uint8_t addr, uint8_t* buffer, size_t len);

/**
 * @brief Writes 1 byte of data to the ADXL372 with SPI
 *
 * @param addr the address of the register
 * @param data the data to write to the register
 * @retval HAL Status, 0 on successful write
 */
static HAL_StatusTypeDef write_register(uint8_t addr, uint8_t data);


static const uint8_t chip_id = 0x01;
static const uint8_t who_am_i = 0x00; 

static SPISettings spiSettings;

int icm45686_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin) {
    if (hspi == NULL || cs_channel == NULL) {
        serialPrintStr("Invalid spi handle or chip select pin for ICM45686");
        return 1;
    }
    // set up the SPI settings
    spiSettings.hspi = hspi;
    spiSettings.cs_channel = cs_channel;
    spiSettings.cs_pin = cs_pin;


    serialPrintStr("Beginning ADXL372 initialization");
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


static int setup_device(bool soft_reset_complete) {
    // datasheet says 6ms to powerup, include some factor of safety (so we using 18!)
    HAL_Delay(18);

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
    // this is a 5ms delay
    HAL_Delay(4);

    // verify chip ID read works
    read_registers(who_am_i, &result, 1);
    if (result != 0x1D) {
        serialPrintStr("\tIMU could not read chip ID");
        return 1;
    }


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