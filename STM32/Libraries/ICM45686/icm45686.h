/*
 * icm45686.h
 *
 *  Created on: Aug 25, 2025
 *      Author: Wlsan
 */

#pragma once
#include <stdbool.h>
#include <math.h>
#include "spi_utils.h"
#include "icm45686_packet.h"

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
int icm45686_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_channel, uint16_t cs_pin);

/**
 * @brief reads acceleration and gyroscope data from the ICM45686, if the data is ready.
 * @note this function should only be called when the interrupt pin is active. This function will
 *       reset the interrupt pin to it's inactive state.
 *
 * @param packet pointer to the IMU packet where the data will be stored
 * @retval 0 if successful, 1 if unsuccessful due to the data not being ready. In this case, the
 *         interrupt pin will still be reset to the inactive state, but no data will be collected.
 */
int icm45686_read_data(ICM45686Packet_t *packet);

/**
 * @brief gets the scale factor of the acceleration readings to convert to g's.
 *
 * @retval float value to divide binary data by to get acceleration in g's.
 */
float icm45686_get_accel_scale_factor(void);

/**
 * @brief gets the scale factor of the gyroscope readings to convert to degrees per second.
 *
 * @retval float value to divide binary data by to get angular rate in degrees per second.
 */
float icm45686_get_gyro_scale_factor(void);

/**
 * @brief sets the SPI settings for the ICM45686
 * 
 * @retval None
 */
void set_spi_icm(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_channel, uint16_t cs_pin);

/**
 * @brief convets raw sensor data from the ICM45686 to board-frame, calibrated, float values
 * @note the offset fields and calibration/rotation matrix field must be set to calibrate.
 *       Default values of [0, 0, 0] offset, and identity matrix will be used otherwise.
 * 
 * @param raw the raw sensor data retrieved from icm45686_read_data()
 * @param out calibrated float values, rotated to board frame
 */
void icm45686_convert_and_calibrate(ICM45686RawData_t *raw, ICM45686BoardReading_t *out);

/**
 * @brief sets the calibration offsets and row-major matrix for the ICM45686 acceleration
 * @note Calculation is done as follows:
 *       (X - b) * A
 *       Where X is the measurement vector in sensor-frame, b is the offsets, and A is the
 *       3x3 row-major rotation matrix for scaling.
 *       The scale matrix should bake-in the rotation matrix for converting from sensor-frame to
 *       board-frame.
 * 
 * @param offsets three float calibration offsets for [x, y, z]
 * @param matrix nine floats for 3x3 row-major matrix for calibration and sensor->board frame
 *               scaling and rotation.
 */
void icm45686_set_accel_calibration(float offsets[3], float matrix[9]);

/**
 * @brief sets the calibration offsets and row-major matrix for the ICM45686 gyroscope
 * @note Calculation is done as follows:
 *       (X - b) * A
 *       Where X is the measurement vector in sensor-frame, b is the offsets, and A is the
 *       3x3 row-major rotation matrix for scaling.
 *       The scale matrix should bake-in the rotation matrix for converting from sensor-frame to
 *       board-frame.
 * 
 * @param offsets three float calibration offsets for [x, y, z]
 * @param matrix nine floats for 3x3 row-major matrix for calibration and sensor->board frame
 *               scaling and rotation.
 */
void icm45686_set_gyro_calibration(float offsets[3], float matrix[9]);