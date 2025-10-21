#include "w25q128jvs.h"
#include "usb_print_debug.h"

/**
 * @brief the SPI settings for the W25Q128JVS to use when accessing device registers
 */
typedef struct {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* cs_channel;
    uint16_t cs_pin;
} SPISettings;

/**
 * @brief Starts up the W25Q128JVS, confirms the SPI read/write functionality is working
 *
 * @retval 0 if successful
 */
static int w25q128jvs_setup_device();

/**
 * @brief Reads data from the W25Q128JVS with SPI
 *
 * @param addr the address of the register
 * @param buffer where the result of the read will be stored
 * @param len the number of bytes to read
 * @retval HAL Status, 0 on successful read
 */
static HAL_StatusTypeDef read_registers(uint8_t addr, uint8_t* buffer, size_t len);

/**
 * @brief Writes 1 byte of data to the W25Q128JVS with SPI
 *
 * @param addr the address of the register
 * @param data the data to write to the register
 * @retval HAL Status, 0 on successful write
 */
static HAL_StatusTypeDef write_register(uint8_t addr, uint8_t data);

static const uint8_t read_id = 0x9F;
static const uint8_t write_enable = 0x06;
static const uint8_t read_status1 = 0x05;
static const uint8_t read_data = 0x03;
static const uint8_t page_program = 0x02;
static const uint8_t sector_erase = 0x20;
static const uint8_t sr1_busy = 0x01;

// W25Q128JVS SPI config settings
static SPISettings spiSettings;

int w25q128jvs_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin) {
    if (hspi == NULL || cs_channel == NULL) {
        serialPrintStr("Invalid spi handle or chip select pin for W25Q128JVS");
        return 1;
    }
    // set up SPI settings
    spiSettings.hspi = hspi;
    spiSettings.cs_channel = cs_channel;
    spiSettings.cs_pin = cs_pin;

    serialPrintStr("Beginning W25Q128JVS initialization");
    // setup device, check spi is properly connected
    if (w25q128jvs_setup_device()) {
        return 1;
    }
    return 0;
}

static int w25q128jvs_setup_device() {
    // 10ms startup time
    HAL_Delay(9);

    uint8_t result = 0;
    read_registers(read_id, &result, 1);
    if (result == 0) {
        serialPrintStr("\tCould not read device ID");
        return 1;
    }
    // not sure yet what the device ID is
    serialPrintStr("\t ID:");
    serialPrintlnInt(result);


    return 0;
}

static HAL_StatusTypeDef read_registers(uint8_t addr, uint8_t* buffer, size_t len) {
    return spi_read(spiSettings.hspi, spiSettings.cs_channel, spiSettings.cs_pin, addr, buffer,
                    len);
}

static HAL_StatusTypeDef write_register(uint8_t addr, uint8_t data) {
    return spi_write(spiSettings.hspi, spiSettings.cs_channel, spiSettings.cs_pin, addr, data);
}
