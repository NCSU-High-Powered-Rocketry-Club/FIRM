#pragma once
#include <stdint.h>
#include <stdbool.h>

#define FIRM_SETTINGS_FREQUENCY_MIN_HZ 1u
#define FIRM_SETTINGS_FREQUENCY_MAX_HZ 1000u
#define SETTINGS_FLASH_BLOCK_SIZE_BYTES 1024u

typedef struct {
    float offset_gs[3];
    float scale_multiplier[9];
} AccelCalibration_t;

typedef struct {
    float offset_dps[3];
    float scale_multiplier[9];
} GyroCalibration_t;

typedef struct {
    float offset_ut[3];
    float scale_multiplier[9];
} MagCalibration_t;

typedef struct {
    AccelCalibration_t icm45686_accel;
    GyroCalibration_t icm45686_gyro;
    MagCalibration_t mmc5983ma_mag;
} CalibrationSettings_t;

typedef struct {
    uint64_t device_uid;
    char device_name[33];
    bool usb_transfer_enabled;
    bool uart_transfer_enabled;
    bool i2c_transfer_enabled;
    bool spi_transfer_enabled;
    char firmware_version[9];
    uint16_t frequency_hz;
} FIRMSettings_t;

extern CalibrationSettings_t calibrationSettings;
extern FIRMSettings_t firmSettings;
