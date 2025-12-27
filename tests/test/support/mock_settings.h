#pragma once
#include <stdint.h>
#include <stdbool.h>


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

// Extern declarations - definitions are in mock_settings.c
extern CalibrationSettings_t calibrationSettings;
extern FIRMSettings_t firmSettings;