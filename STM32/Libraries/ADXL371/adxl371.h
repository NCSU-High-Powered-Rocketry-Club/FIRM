/*
 * adxl371.h
 *
 *  Created on: Jan 29, 2026
 *      Author: Adapted by ERM from code written by Wlsan
 */

#pragma once
#include "adxl371_packet.h"
#include "spi_utils.h"
#include <math.h>
#include <stdbool.h>

/**
 * @brief ensures SPI read/write is working to the ADXL371, and configures register settings.
 *
 * @note Sets ODR to 160hz, data ready to INT1 (Active Low), Disables FIFO, disables High and Low
 * pass Filters,  - Turns off oustosleep and linkloop, sets chip to low noise operation.
 *
 * @param hspi pointer to the SPI channel that the ADXL371 is connected to
 * @param cs_channel specifies the GPIO channel that the chip select pin is connected to.
 * @param cs_pin specifies the GPIO pin that the chip select pin is connected to.
 * @retval 0 upon success
 */
int adxl371_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_channel, uint16_t cs_pin);

/**
 * @brief sets the SPI settings for the ADXL371 without re-initializing the device
 *
 * @param hspi pointer to the SPI channel that the ADXL371 is connected to
 * @param cs_channel specifies the GPIO channel that the chip select pin is connected to.
 * @param cs_pin specifies the GPIO pin that the chip select pin is connected to.
 */
void set_spi_adxl(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_channel, uint16_t cs_pin);

/**
 * @brief reads the acceleration measurements from the ADXL371
 *
 * @note used a single read statement to batch read from xdata_h to zdata_l (0x08 to 0x0D)
 *
 * @param packet pointer to the accelerometer packet where the data will be stored
 * @retval 0 upon success, 1 if no new data is ready yet
 */
int adxl371_read_data(ADXL371RawData_t *packet);

/**
 * @brief returns the scale factor for the ADXL371 accelerometer
 * @note the ADXL371 is fixed at +-200g, scale factor is 4096/400 = 10.24
 *
 * @retval scale factor as a float
 */
float adxl371_get_accel_scale_factor(void);

/**
 * @brief convets raw sensor data from the ADXL371 to board-frame, calibrated, float values
 * @note the offset fields and calibration/rotation matrix field must be set to calibrate.
 *       Default values of [0, 0, 0] offset, and identity matrix will be used otherwise.
 * 
 * @param raw the raw sensor data retrieved from adxl371_read_data()
 * @param out calibrated float values, rotated to board frame
 */
void adxl371_convert_and_calibrate(ADXL371RawData_t *raw, ADXL371BoardReading_t *out);

/**
 * @brief sets the calibration offsets and row-major matrix for the ADXL371 acceleration
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
void adxl371_set_calibration(float offsets[3], float matrix[9]);