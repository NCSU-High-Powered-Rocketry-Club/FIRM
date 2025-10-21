#pragma once
#include "w25q128jv.h"
#include "spi_utils.h"
#include <stdbool.h>

/**
 * Accelerometer calibration coefficients
 */
typedef struct {
    float offset_gs[3];
    float scale_multiplier[3];
} AccelCalibration_t;

/**
 * Gyroscope calibration coefficients
 */
typedef struct {
    float offset_dps[3];
    float scale_multiplier[3];
} GyroCalibration_t;

/**
 * Magnetometer calibration coefficients
 */
typedef struct {
    float offset_ut[3];
    float scale_multiplier[3];
} MagCalibration_t;

/**
 * Calibration settings for FIRM's currently used sensors
 */
typedef struct {
    AccelCalibration_t icm45686_accel;
    GyroCalibration_t icm45686_gyro;
    MagCalibration_t mmc5983ma_mag;
} CalibrationSettings_t;

/**
 * User-defined settings for FIRM, saved in flash memory and reloaded on boot.
 */
typedef struct {
    // TODO: finish python bindings and make real settings
    bool accel_enabled;
} FIRMSettings_t;

// global declaration of FIRM Settings to be used elsewhere in project
extern FIRMSettings_t FIRMSettings;

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
int settings_init(SPI_HandleTypeDef* flash_hspi, GPIO_TypeDef* flash_cs_channel, uint16_t flash_cs_pin);

