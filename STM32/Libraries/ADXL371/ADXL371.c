#include "ADXL371.h"
#include "spi_utils.h"

/**
 * @brief the SPI settings for the ADXL372 to use when accessing device registers
 */

typedef struct{
    SPI_TypeDef* hspi;
    GPIO_TypeDef* cs_channel;
    uint16_t cs_pin;
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


static const uint8_t chip_id = 0x02;
static const uint8_t who_am_i = 0x00; 
static const uint8_t fifo_samples =0x39;
static const uint8_t reset = 0x41;
static const uint8_t status = 0x41;
static const uint8_t timing = 0x3D;
static const uint8_t int1_map = 0x3B;
static const uint8_t int2_map = 0x3C;
static const uint8_t power_ctl = 0x3F;
static const uint8_t measure = 0x3E;
static const uint8_t fifo_clt= 0x3A;

static const uint8_t xdata_h= 0x08;
static const uint8_t xdata_l= 0x09;
static const uint8_t ydata_h= 0x0A;
static const uint8_t ydata_l= 0x0B;
static const uint8_t zdata_h= 0x0C;
static const uint8_t zdata_l= 0x0D;

static const float scale_factor_accel = 0.0976;

static SPISettings spiSettings;

int icm45686_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin) {
   
   
    if (hspi == NULL || cs_channel == NULL) {
        serialPrintStr("Invalid spi handle or chip select pin for ADXL371");
        return 1;
    }
    // set up the SPI settings
    spiSettings.hspi = hspi;
    spiSettings.cs_channel = cs_channel;
    spiSettings.cs_pin = cs_pin;


    serialPrintStr("Beginning ADXL371 initialization");
    // sets up the IMU in SPI mode and ensures SPI is working
    if (setup_device(false)) return 1;

    // do a soft-reset of the sensor's settings
    serialPrintStr("\tIssuing ADXL371 software reset...");
    write_register(reset, 0x52);
    // verify correct setup again
    if (setup_device(true)) return 1;

    //sets ODR to 1280
    write_register(timing,0b01000000);

   // Accelerometer is fixed to +-200g
    


    //bit 3-1 in int1_map are reserved so I am masking. To avoid modifying those bits
    uint8_t value;
    //sets all non reserved bits to 0 thus disabeling interrups before config
    read_registers(int1_map, &value, 1);
    value = value & 0b00001110;
    write_register(int1_map, value);
    //Map Data Ready to Int1 and Active low
    read_registers(int1_map, &value,1);
    value = value & 0b00001110;
    value = value | 0b10000001;
    write_register(int1_map, value);

    //Disable FIFO
    read_registers(fifo_clt, &value,1);
    value = value & 0b11111001;
    value = value | 0b00000000;
    write_register(fifo_clt, value);

    //Turning off High Pass Filter and Low Pass Filter
    read_registers(power_ctl, &value,1);
    value = value & 0b11110011;
    value = value | 0b00001100;
    write_register(power_ctl, value);

    //Configures measurement settings (Antialiasing 640), normal noise operation
    //Turn off autosleep, link loop. Set low noise operation
    write_register(measure, 0b00001010);


    // delay for accel to get ready
    HAL_Delay(12);

    //set adxl to measure more
    read_registers(power_ctl, &value,1);
    value = value & 0b1111100;
    value = value | 0b00000011;
    write_register(power_ctl, value);
    
    serialPrintStr("\tADXL371 startup successful!");
    

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
    if (result != 0xFA) {
        serialPrintStr("\tIMU could not read chip ID");
        return 1;
    }

    //write check

    read_registers(fifo_samples, &result, 1);

    if (result != 0x80) {
        serialPrintStr("\tIMU could not read fifo samples");
        return 1;
    }

    write_register(fifo_samples, 0x00);

    if (result != 0x00) {
        serialPrintStr("\tIMU could not write fifo samples");
        return 1;
    }

    write_register(fifo_samples, 0x80);
    

    if (soft_reset_complete) {
        // Check bit 1 (soft reset bit) is set back to 0
        read_registers(status, &result, 1);
        if ((result & 0x80) != 0) {
            serialPrintStr("\tSoftware reset failed!");
            return 1;
        }
    }
    return 0;

}

float adxl371_get_pressure_scale_factor(void) {
    return scale_factor_accel;
}

int adxl371_read_data(ADXL371Packet_t* packet) {
    // clear interrupt (pulls interrupt back up high) and verify new data is ready
    uint8_t data_ready = 0;
    read_registers(status, &data_ready, 1);
    if (data_ready & 0b00000001) { // bit 0 (LSB) will be 1 if new data is ready
        //Burst read xdata_h to zdata_l (0x08 to 0x0D) into packet
        read_registers(xdata_h,(uint8_t*)packet, 6);
        return 0;
    }
    return 1;
}

static HAL_StatusTypeDef read_registers(uint8_t addr, uint8_t* buffer, size_t len) {
    return spi_read(spiSettings.hspi, spiSettings.cs_channel, spiSettings.cs_pin, addr, buffer,
                    len);
}


static HAL_StatusTypeDef write_register(uint8_t addr, uint8_t data) {
    return spi_write(spiSettings.hspi, spiSettings.cs_channel, spiSettings.cs_pin, addr, data);
}