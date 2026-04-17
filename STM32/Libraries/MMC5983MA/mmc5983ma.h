/*
 * mmc5983ma.h
 *
 *  Created on: Sep 8, 2025
 *      Author: Wlsan
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <spi_utils.h>
#include "mmc5983ma_packet.h"

/**
 * @brief ensures SPI read/write is working to the MMC5983MA, and configures register settings
 *
 * @retval 0 upon success
 */
int mmc5983ma_init(void);

/**
 * @brief reads data from the MMC5983MA
 *
 * @param packet pointer to the Magnetometer packet where the data will be stored
 *
 * @ret error status, returns 0 on success, 1 on failure
 */
int mmc5983ma_read_data(MMC5983MARawData_t* packet);

/**
 * @brief gets the scale factor of the magnetometer readings to convert to microteslas.
 * @note this value is the same scale factor used for all axes of the magnetometer.
 * 
 * @retval float value to divide binary data by to get magnetic field in microteslas.
 */
float mmc5983ma_get_magnetic_field_scale_factor(void);

/**
 * @brief convets raw sensor data from the MMC5983MA to board-frame, calibrated, float values
 * @note the offset fields and calibration/rotation matrix field must be set to calibrate.
 *       Default values of [0, 0, 0] offset, and identity matrix will be used otherwise.
 * 
 * @param raw the raw sensor data retrieved from mmc5983ma_read_data()
 * @param out calibrated float values, rotated to board frame
 */
void mmc5983ma_convert_and_calibrate(MMC5983MARawData_t *raw, MMC5983MABoardReading_t *out);

/**
 * @brief sets the calibration offsets and row-major matrix for the MMC5983MA magnetic field
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
void mmc5983ma_set_calibration(float offsets[3], float matrix[9]);

/**
 * @brief sets the SPI settings for the MMC5983MA
 * 
 * @retval None
 */
void set_spi_mmc(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_channel, uint16_t cs_pin);