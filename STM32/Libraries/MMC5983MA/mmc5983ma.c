/*
 * mmc5983ma.c
 *
 *  Created on: Sep 8, 2025
 *      Author: Wlsan
 */
#include "mmc5983ma.h"

/**
 * @brief the SPI settings for the MMC5983MA to use when accessing device registers
 */
typedef struct {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_channel;
  uint16_t cs_pin;
} SPISettings;

/**
 * @brief Starts up and resets the magnetometer, confirms the SPI read/write functionality is
 * working
 *
 * @param soft_reset_complete if this is a setup after a soft reset is complete
 * @retval 0 if successful
 */
static int setup_device(bool soft_reset_complete);

/**
 * @brief Reads data from the MMC5983MA with SPI
 *
 * @param reg_addr the address of the register
 * @param buffer where the result of the read will be stored
 * @param len the number of bytes to read
 * @retval HAL Status, 0 on successful read
 */
static HAL_StatusTypeDef read_registers(uint8_t reg_addr, uint8_t *buffer, size_t len);

/**
 * @brief Writes 1 byte of data to the MMC5983MA with SPI
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

static SPISettings spiSettings;

/** Calibration and orientation values for converting raw sensor data to board-frame floats */
static float calibration_offsets[3] = {0};
static float calibration_matrix[9] = {1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F};

void set_spi_mmc(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_channel, uint16_t cs_pin) {
  spiSettings.hspi = hspi;
  spiSettings.cs_channel = cs_channel;
  spiSettings.cs_pin = cs_pin;
}

int mmc5983ma_init(void) {
  if (spiSettings.hspi == NULL || spiSettings.cs_channel == NULL) {
    // Invalid SPI handle
    return 1;
  }


  // Beginning MMC5983MA initialization
  // sets up the magnetometer in spi mode and ensures spi is working
  if (setup_device(false))
    return 1;

  // initiating a software reset
  write_register(internal_control1, 0b10000000);

  // verify correct setup again
  if (setup_device(true))
    return 1;

  // Perform SET and RESET to get rid of magnet build up
  // Perform SET
  write_register(internal_control0, 0b00001000);
  HAL_Delay(10);
  // Perform RESET
  write_register(internal_control0, 0b00010000);
  HAL_Delay(10);

  // enable interrupt pin
  write_register(internal_control0, 0b00000100);
  // set bandwidth to 200hz (4ms measurement time)
  write_register(internal_control1, 0b00000001);
  // enable continuous measurement mode at 200hz
  write_register(internal_control2, 0b00001110);

  // MMC5983MA startup successful
  return 0;
}

int mmc5983ma_read_data(MMC5983MARawData_t *packet) {
  uint8_t data_ready = 0;
  // read status register to make sure data is ready
  read_registers(status, &data_ready, 1);
  if (data_ready & 0x01) { // data ready bit is 0x01
    // manually clear the interrupt signal
    write_register(status, 0b00000001);

    // burst read 7 bytes of data from the first data register
    // first two bytes are magnetometer x
    // next two are magnetometer y
    // next two are magnetometer z
    // last byte is 2 bits of LSB x, 2 bits of LSB y, 2 bits of LSB z, and 2 reserved bits
    read_registers(x_out0, (uint8_t *)packet, 7);

    return 0;
  }
  return 1;
}

float mmc5983ma_get_magnetic_field_scale_factor(void) {
  // since the sensor has write-only config registers, this means we can't read from it
  // to determine which FS range we are at. I really don't care enough to set up shadow
  // register memory for this driver, so i'm just going to return the value we are using.
  return scale_factor;
}

void mmc5983ma_convert_and_calibrate(MMC5983MARawData_t *raw, MMC5983MABoardReading_t *out) {
  // extract magnetic field bytes as 32 bit integer, which uses two's complement
  int32_t mag_binary_x = (((int32_t)(raw->mag_x_msb) << 10) | ((int32_t)raw->mag_x_mid << 2) |
                          ((int32_t)(raw->mag_xyz_lsb >> 6))) -
                         data_num_lsb_bits;

  int32_t mag_binary_y = (((int32_t)(raw->mag_y_msb) << 10) | ((int32_t)raw->mag_y_mid << 2) |
                          ((int32_t)((raw->mag_xyz_lsb & 0b00110000) >> 4))) -
                         data_num_lsb_bits;

  int32_t mag_binary_z = (((int32_t)(raw->mag_z_msb) << 10) | ((int32_t)raw->mag_z_mid << 2) |
                          ((int32_t)((raw->mag_xyz_lsb & 0b00001100) >> 2))) -
                         data_num_lsb_bits;

  // convert to float in SI units (microtesla) and subtract by magnetometer calibration offsets
  float mag_float_x = ((float)mag_binary_x) / scale_factor - calibration_offsets[0];
  float mag_float_y = ((float)mag_binary_y) / scale_factor - calibration_offsets[1];
  float mag_float_z = ((float)mag_binary_z) / scale_factor - calibration_offsets[2];

  // apply 3x3 scaling matrix to each value
  out->magnetic_field_x_microteslas = mag_float_x * calibration_matrix[0] +
                                      mag_float_y * calibration_matrix[3] +
                                      mag_float_z * calibration_matrix[6];
  out->magnetic_field_y_microteslas = mag_float_x * calibration_matrix[1] +
                                      mag_float_y * calibration_matrix[4] +
                                      mag_float_z * calibration_matrix[7];
  out->magnetic_field_z_microteslas = mag_float_x * calibration_matrix[2] +
                                      mag_float_y * calibration_matrix[5] +
                                      mag_float_z * calibration_matrix[8];
}

void mmc5983ma_set_calibration(float offsets[3], float matrix[9]) {
  for (int i = 0; i < 3; i++) {
    calibration_offsets[i] = offsets[i];
  }
  for (int i = 0; i < 9; i++) {
    calibration_matrix[i] = matrix[i];
  }
}

int setup_device(bool soft_reset_complete) {
  HAL_Delay(14); // 15ms power-on time
  uint8_t result = 0;
  // perform dummy read as required by datasheet
  HAL_StatusTypeDef hal_status = read_registers(product_id1, &result, 1);
  if (hal_status) {
    return 1;
  }
  // give device enough time to switch to correct mode
  // this is a 1ms delay
  HAL_Delay(0);

  read_registers(product_id1, &result, 1);
  if (result != product_id_val) {
    // MMC5983MA could not read Product ID
    return 1;
  }

  // unlike the other sensors, the registers are read-only or write-only, so the startup
  // write test cannot be done, because the value of the written register cannot be verified
  // with a read

  if (soft_reset_complete) {
    // check that bit 7 (sw_rst) is back to 0
    read_registers(internal_control1, &result, 1);
    if (result & 0x80) {
      // MMC5983MA did not complete software reset
      return 1;
    }
  }
  return 0;
}

static HAL_StatusTypeDef read_registers(uint8_t addr, uint8_t *buffer, size_t len) {
  addr |= 0x80;
  return spi_read(spiSettings.hspi, spiSettings.cs_channel, spiSettings.cs_pin, addr, buffer, len);
}

static HAL_StatusTypeDef write_register(uint8_t addr, uint8_t data) {
  return spi_write(spiSettings.hspi, spiSettings.cs_channel, spiSettings.cs_pin, addr, data);
}