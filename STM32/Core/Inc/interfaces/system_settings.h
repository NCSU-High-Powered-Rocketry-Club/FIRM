#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float offset[3];
  float scale_matrix[9];
} Calibration_t;

/**
 * User-defined settings and calibration for FIRM, saved in flash memory and reloaded on boot.
 */
typedef struct __attribute__((packed)) {
  uint64_t device_uid; // UID of settings storage device
  char device_name[32]; // user configurable device name
  bool usb_transfer_enabled;
  bool uart_transfer_enabled;
  bool i2c_transfer_enabled;
  bool spi_transfer_enabled;
  char firmware_version[8];
  uint16_t frequency_hz;
  Calibration_t accel_cal;
  Calibration_t gyro_cal;
  Calibration_t mag_cal;
  Calibration_t high_g_cal;
} SystemSettings_t;