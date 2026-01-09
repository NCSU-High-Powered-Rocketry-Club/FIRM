#pragma once
#include <stdint.h>
#include <stdbool.h>

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
} FIRMSettings_t;

// global declaration of FIRM Settings and calibration settings to be used elsewhere in project
extern FIRMSettings_t firmSettings;
extern CalibrationSettings_t calibrationSettings;