#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef TEST
    #include "stm32_hal_stubs.h" // from tests/test/support
    #include "w25q128jv_stubs.h" // stubs in tests/test/support
    #include "serial_print.h" // mocked serialPrintStr
    #include "hal_gpio.h" // mocked HAL_GPIO_ReadPin 
#else
    #include <w25q128jv.h>
    #include "usb_print_debug.h"
#endif

#define FIRM_SETTINGS_FREQUENCY_MIN_HZ 1u
#define FIRM_SETTINGS_FREQUENCY_MAX_HZ 1000u

/**
 * We store our settings in sector 0, on the first 1024-byte block.
 */
#define SETTINGS_FLASH_BLOCK_SIZE_BYTES 1024u

/**
 * Accelerometer calibration coefficients
 */
typedef struct {
    float offset_gs[3];
    float scale_multiplier[9];
} AccelCalibration_t;

/**
 * Gyroscope calibration coefficients
 */
typedef struct {
    float offset_dps[3];
    float scale_multiplier[9];
} GyroCalibration_t;

/**
 * Magnetometer calibration coefficients
 */
typedef struct {
    float offset_ut[3];
    float scale_multiplier[9];
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
    uint64_t device_uid; // flash chip UID
    char device_name[33]; // 32 character limit, plus null-terminator
    bool usb_transfer_enabled;
    bool uart_transfer_enabled;
    bool i2c_transfer_enabled;
    bool spi_transfer_enabled;
    char firmware_version[9]; // 8 character limit, plus null-terminator
    uint16_t frequency_hz;
} FIRMSettings_t;

extern FIRMSettings_t firmSettings;
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
int settings_init(SPI_HandleTypeDef* flash_hspi, GPIO_TypeDef* flash_cs_channel, uint16_t flash_cs_pin);

/**
 * Writes the calibration settings to the flash chip.
 *
 * @param calibration_settings pointer to calibration settings to write
 */
bool settings_write_calibration_settings(CalibrationSettings_t* calibration_settings);

/**
 * Writes the firm settings to the flash chip.
 *
 * @param firm_settings pointer to firm settings to write
 */
bool settings_write_firm_settings(FIRMSettings_t* firm_settings);

