#include "ina219.h"
#include <stdint.h>
#include "ina219_packet.h"


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
static HAL_StatusTypeDef read_registers(uint8_t reg_addr, uint16_t *buffer, size_t len);

/**
 * @brief Writes 1 byte of data to the MMC5983MA with I2C
 *
 * @param reg_addr the address of the register
 * @param data the data to write to the register
 * @retval HAL Status, 0 on successful write
 */
static HAL_StatusTypeDef write_registers(uint8_t reg_addr, uint16_t data);


//Since max expected current is 1A, the current_lsb is 1/(2)^15
//Shunt resistor is .01 ohms. So according to the data sheet, the calibration calc is 
//133332
static const uint8_t configuration = 0x00; //config defaults to 399F
static const uint8_t shunt_voltage = 0x01;
static const uint8_t bus_voltage = 0x02;
static const uint8_t power = 0x03;
static const uint8_t current = 0x04;
static const uint8_t calibration =0x05;


static I2CSettings i2cSettings;

void set_spi_ina(I2C_HandleTypeDef *hi2c, uint8_t device_i2c_addr) {
  i2cSettings.hi2c = hi2c;
  i2cSettings.dev_addr = device_i2c_addr;
}

int ina219_init(I2C_HandleTypeDef *hi2c, uint8_t device_i2c_addr) {
  if (hi2c == NULL) {
    serialPrintStr("Invalid i2c handle for INA219");
    return 1;
  }

  // configure i2c settings
  i2cSettings.hi2c = hi2c;
  i2cSettings.dev_addr = device_i2c_addr;
  serialPrintStr("Beginning INA219 initialization");

  // sets up the magnetometer in I2C mode and ensures I2C is working
  if (setup_device(false))
    return 1;

    // initiating a software reset
  serialPrintStr("\tIssuing INA219 software reset...");
  write_registers(configuration, 0b1011100110011111);
  
  // verify correct setup again
  if (setup_device(true))
    return 1;
  // Sets the voltage range to 32FSR, PGA to +-320 with a gain of /8
  //BADC (Voltage Bus) SADC(shunt voltage)  is set to a voltage resolution to 12 bits and a sample size to 16 samples
  //  sets Shunt and voltage bus, continuous
  write_registers(configuration, 0b0011111001100111);

  //writes calculated calibration value. (Need to recalculate)
  write_registers(calibration, 0x0000);

  serialPrintStr("\tINA219 startup successful!");
  return 0;
}

int setup_device(bool soft_reset_complete) {
  HAL_Delay(14); // 15ms power-on time
  uint16_t result = 0;

  // perform dummy read as required by datasheet
  HAL_StatusTypeDef hal_status = read_registers(configuration, &result, 2);
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

  if (soft_reset_complete) {

    read_registers(configuration, &result, 2);
    if (result == 0b1011100110011111) { //the lsb of config should flip to low after reset
      serialPrintStr("\tINA219 did not complete software reset");
      return 1;
    }
  }


  //read check
  read_registers(configuration, &result, 2);
  if (result != 0x399F) {
    serialPrintStr("\t INA219 could not read");
    return 1;
  }

  //write check
  //writting to a bit which is not in use
    write_registers(calibration, 0x799E);
    read_registers(calibration, &result, 2);
    if (result != 0x799E ){
      serialPrintStr("\t INA219 could not write");
      return 1;
    }
  
  return 0;
}

int ina219_read_data(INA219Packet_t* packet){
  read_registers(shunt_voltage,packet->shunt_voltage ,2);
  read_registers(bus_voltage,packet->bus_voltage, 2);
  read_registers(power,packet->power, 2);
  read_registers(current,packet->current, 2);

  return 0;
}

static HAL_StatusTypeDef read_registers(uint8_t reg_addr, uint16_t *buffer, size_t len) {

  HAL_StatusTypeDef HAL = HAL_I2C_Mem_Read(i2cSettings.hi2c, (uint16_t)(i2cSettings.dev_addr << 1),
                          (uint8_t)reg_addr, I2C_MEMADD_SIZE_8BIT, buffer, len, 100);
  
  //Shift MSB and LSB to big Endian
  uint16_t MSB = 0xFF00;
  uint16_t LSB = 0x00FF;

  MSB = *buffer & MSB;
  LSB = *buffer & LSB;

  *buffer = 0x0000;



  MSB=MSB>>8;
  LSB=LSB<<8;


  *buffer = *buffer | MSB;
  *buffer = *buffer | LSB;
  

  //----------------- 
  return HAL;
}

static HAL_StatusTypeDef write_registers(uint8_t reg_addr, uint16_t data) {
  //Shift MSB and LSB to big Endian
  uint16_t MSB = 0xFF00;
  uint16_t LSB = 0x00FF;

  MSB = data & MSB;
  LSB = data & LSB;

  data = 0x0000;

  MSB =MSB>>8;
  LSB= LSB<<8;

  data = data | MSB;
  data = data | LSB;




  return HAL_I2C_Mem_Write(i2cSettings.hi2c, (uint16_t)(i2cSettings.dev_addr << 1),
                           (uint8_t)reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 2, 100);
}