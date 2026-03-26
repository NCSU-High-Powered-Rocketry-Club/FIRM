#pragma once

#include "led.h"
#include <stdbool.h>
#include <stdint.h>
#include <w25q128jv.h>

#define FIRM_SETTINGS_FREQUENCY_MIN_HZ 1u
#define FIRM_SETTINGS_FREQUENCY_MAX_HZ 1000u
#define FIRM_FIRMWARE_VERSION "v2.1.0"

/**
 * We store our settings in sector 0, on the first 1024-byte block.
 */
#define SETTINGS_FLASH_BLOCK_SIZE_BYTES 512



/**
 * Global instances of the FIRM settings.
 */
extern FIRMSettings_t firmSettings;
/**
 * Global instance of the calibration settings.
 */
extern CalibrationSettings_t calibrationSettings;

/**
 * @brief sets up the settings module by initializing the flash chip and ensuring the device
 *        has existing calibration data. If existing, saves to settings struct
 * @note If calibration data does not exist, device needs to be given to repo owner to calibrate
 *
 * @param flash_hspi pointer to the SPI channel that the settings flash chip is connected to
 * @param flash_cs_channel specifies the GPIO channel that the chip select pin is connected to.
 * @param flash_cs_pin specifies the GPIO pin that the chip select pin is connected to.
 * @retval 0 upon success
 */
int settings_init(SPI_HandleTypeDef *flash_hspi, GPIO_TypeDef *flash_cs_channel,
                  uint16_t flash_cs_pin);

/**
 * Writes the calibration settings to the flash chip.
 *
 * @param calibration_settings pointer to calibration settings to write
 */
bool settings_write_calibration_settings(AccelCalibration_t *accel_cal_settings,
                                         GyroCalibration_t *gyro_cal_settings,
                                         MagCalibration_t *mag_cal_settings,
                                         ADXLAccelCalibration_t *adxl371_accel_cal_settings);

/**
 * Writes the firm settings to the flash chip.
 *
 * @param firm_settings pointer to firm settings to write
 */
bool settings_write_firm_settings(FIRMSettings_t *firm_settings);

/**
 * Writes mock calibration and firmware settings to sector 2 of the flash chip.
 *
 * @param firm_settings pointer to firm settings to write
 * @param calibration_settings pointer to calibration settings to write
 */
bool settings_write_mock_settings(FIRMSettings_t *firm_settings,
                                  CalibrationSettings_t *calibration_settings);
