#include "ina219.h"


/**
 * @brief the I2C settings for the INA219 to use when accessing device registers
 */
typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint8_t dev_addr; // 7 bit i2c address for the device
} I2CSettings;

/**
 * @brief Starts up and resets the magnetometer, confirms the I2C read/write functionality is
 * working
 *
 * @param soft_reset_complete if this is a setup after a soft reset is complete
 * @retval 0 if successful
 */
static int setup_device(bool soft_reset_complete);

/**
 * @brief Reads data from the INA219 with I2C
 *
 * @param reg_addr the address of the register
 * @param buffer where the result of the read will be stored
 * @param len the number of bytes to read
 * @retval HAL Status, 0 on successful read
 */
static HAL_StatusTypeDef read_registers(uint16_t reg_addr, uint16_t *buffer, size_t len);

/**
 * @brief Writes 1 byte of data to the MMC5983MA with I2C
 *
 * @param reg_addr the address of the register
 * @param data the data to write to the register
 * @retval HAL Status, 0 on successful write
 */
static HAL_StatusTypeDef write_register(uint16_t reg_addr, uint16_t data);

static const uint16_t configuration = 0x00; //config defaults to 399F

int setup_device(bool soft_reset_complete) {
  HAL_Delay(14); // 15ms power-on time
  uint16_t result = 0;
  // perform dummy read as required by datasheet
  HAL_StatusTypeDef hal_status = read_registers(configuration, &result, 1);
  if (hal_status) {
    switch (hal_status) {
    case HAL_BUSY:
      serialPrintStr("\tI2C handle currently busy, unable to read");
      break;
    case HAL_ERROR:
      serialPrintStr("\tI2C read transaction failed during dummy read");
      break;
    default:
      break;
    }
    return 1;
  }
  // give device enough time to switch to correct mode
  // this is a 1ms delay
  HAL_Delay(0);

  read_registers(configuration, &result, 1);
  if (result != 0x339F) {
    serialPrintStr("\t INA219 could not read");
    return 1;
  }

  //write check
  //writting to a bit which is not in use
    write_register(configuration, 0x733F);
    read_registers(configuration, &result, 1);
    if (result != 0x733F ){
      serialPrintStr("\t INA219 could not write");
      return 1;
    }

  if (soft_reset_complete) {
    // check that bit 7 (sw_rst) is back to 0
    read_registers(internal_control1, &result, 1);
    if (result & 0x80) {
      serialPrintStr("\tMMC5983MA did not complete software reset");
      return 1;
    }
  }
  return 0;
}


static HAL_StatusTypeDef read_registers(uint16_t reg_addr, uint16_t *buffer, size_t len) {
  return HAL_I2C_Mem_Read(i2cSettings.hi2c, (uint16_t)(i2cSettings.dev_addr << 1),
                          (uint16_t)reg_addr, I2C_MEMADD_SIZE_16BIT, buffer, len, 100);
}

static HAL_StatusTypeDef write_register(uint16_t reg_addr, uint16_t data) {
  return HAL_I2C_Mem_Write(i2cSettings.hi2c, (uint16_t)(i2cSettings.dev_addr << 1),
                           (uint16_t)reg_addr, I2C_MEMADD_SIZE_16BIT, &data, 1, 100);
}