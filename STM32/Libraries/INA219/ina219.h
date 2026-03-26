#pragma once
#include "usb_print_debug.h"
#include <stdbool.h>
#include <stdint.h>
#include "ina219_packet.h"

/**
 * @brief sets up the INA219 magnetometer with the intended settings for flight
 *
 * @param hi2c pointer to the i2c handle used for the device
 * @param device_i2c_addr the 7-bit INA219's i2c device address, should be 0x40 if A0 and A1 is connected to ground
 * @ret error status, returns 0 on success, 1 on failure
 */
int ina219_init(I2C_HandleTypeDef *hi2c, uint8_t device_i2c_addr);

/**
 * @brief reads data from the INA219
 *
 * @param packet pointer to the packet where the data will be stored
 *
 * @ret error status, returns 0 on success
 */
int ina219_read_data(INA219Packet_t* packet);
