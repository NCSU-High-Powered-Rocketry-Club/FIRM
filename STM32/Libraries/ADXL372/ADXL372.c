
#include <spi_utils.h>

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


static const uint8_t chip_id = 0xFA;
static const uint8_t status_reg =;

static SPISettings spiSettings;

static HAL_StatusTypeDef read_registers(uint8_t addr, uint8_t* buffer, size_t len) {
    return spi_read(spiSettings.hspi, spiSettings.cs_channel, spiSettings.cs_pin, addr, buffer,
                    len);
}


static HAL_StatusTypeDef write_register(uint8_t addr, uint8_t data) {
    return spi_write(spiSettings.hspi, spiSettings.cs_channel, spiSettings.cs_pin, addr, data);
}