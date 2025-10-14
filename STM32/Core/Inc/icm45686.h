/*
 * icm45686.h
 *
 *  Created on: Aug 25, 2025
 *      Author: Wlsan
 */

#pragma once
#include "packets.h"
#include "usb_print_debug.h"
#include <stdbool.h>
#include <math.h>


/**
 * @brief Indirect Register type enumeration
 */
typedef enum IREGMap { IMEM_SRAM, IPREG_BAR, IPREG_SYS1, IPREG_SYS2, IPREG_TOP1 } IREGMap_t;

/**
 * @brief initializes ICM45686 IMU settings
 *
 * @note disables the AUX1 channel, sets to 32g cap, 1600hz ODR, 4000 dps cap, and enables
 * 		 interrupt pin on data_ready. Interrupt is set to active-low, latching, and
 * push-pull.
 *
 * @param hspi specifies the SPI channel that the ICM45686 is connected to.
 * @param cs_channel specifies the GPIO channel that the chip select pin is connected to.
 * @param cs_pin specifies the GPIO pin that the chip select pin is connected to.
 * @retval 0 if successful, 1 if unsuccessful due to a register being written to incorrectly
 *         or if a register did not output the expected value when read.
 */
int icm45686_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_channel, uint16_t cs_pin);

/**
 * @brief reads acceleration and gyroscope data from the ICM45686, if the data is ready.
 * @note this function should only be called when the interrupt pin is active. This function will
 *       reset the interrupt pin to it's inactive state.
 *
 * @param packet pointer to the IMU packet where the data will be stored
 * @retval 0 if successful, 1 if unsuccessful due to the data not being ready. In this case, the
 *         interrupt pin will still be reset to the inactive state, but no data will be collected.
 */
int icm45686_read_data(IMUPacket_t* packet);

/**
 * @brief gets the scale factor of the acceleration readings to convert to g's.
 * 
 * @retval float value to divide binary data by to get acceleration in g's. Returns -1 if sensor
 *         is not initialized yet.
 */
float icm45686_get_accel_scale_factor(void);

/**
 * @brief gets the scale factor of the gyroscope readings to convert to radians per second.
 * 
 * @retval float value to divide binary data by to get angular rate in radians per second.
 *         Returns -1 if sensor is not initialized yet.
 */
float icm45686_get_gyro_scale_factor(void);
