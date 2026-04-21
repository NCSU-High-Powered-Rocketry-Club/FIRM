#pragma once
#include <stdbool.h>
#include <stdint.h>

#define FIRM_DEVICE_NAME_LENGTH 32
#define FIRM_FIRMWARE_VERSION_LEN 8
#define FIRM_SETTINGS_FREQUENCY_MIN_HZ 1u
#define FIRM_SETTINGS_FREQUENCY_MAX_HZ 1000u
#define FIRM_FIRMWARE_VERSION "v2.2.0"

typedef struct {
  float offset[3];
  float scale_matrix[9];
} Calibration_t;

/**
 * User-defined settings and calibration for FIRM, saved in flash memory and reloaded on boot.
 */
typedef struct __attribute__((packed)) {
  uint64_t device_uid; // UID of settings storage device
  char device_name[FIRM_DEVICE_NAME_LENGTH]; // user configurable device name
  bool usb_transfer_enabled;
  bool uart_transfer_enabled;
  bool i2c_transfer_enabled;
  bool spi_transfer_enabled;
  char firmware_version[FIRM_FIRMWARE_VERSION_LEN];
  uint16_t frequency_hz;
  Calibration_t accel_cal;
  Calibration_t gyro_cal;
  Calibration_t mag_cal;
  Calibration_t high_g_cal;
} SystemSettings_t;

typedef struct {
  uint16_t frequency_hz;
  char device_name[FIRM_DEVICE_NAME_LENGTH];
  bool usb_transfer_enabled;
  bool uart_transfer_enabled;
  bool i2c_transfer_enabled;
  bool spi_transfer_enabled;
} DeviceConfig_t;

typedef struct {
  uint64_t device_uid;
  char firmware_version[FIRM_FIRMWARE_VERSION_LEN];
} DeviceInfo_t;