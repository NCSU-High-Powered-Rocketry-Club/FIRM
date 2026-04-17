#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Non-owning view of raw sensor bytes.
 *
 * @note Memory is owned by the concrete sensor context and may be overwritten
 *       by the next read call.
 */
typedef struct {
  const uint8_t *bytes;
  size_t len;
} SensorRawView_t;

/**
 * @brief Generic converted barometer sample used by processing code.
 */
typedef struct {
  float pressure_pa;
  float temperature_celsius;
} BarometerSample_t;

typedef int (*SensorInitFn_t)(void *ctx);
typedef int (*SensorReadFn_t)(void *ctx);
typedef SensorRawView_t (*SensorGetRawFn_t)(const void *ctx);

typedef int (*BarometerConvertFn_t)(const void *ctx, BarometerSample_t *out);

typedef struct {
  void *ctx;
  SensorInitFn_t init;
  SensorReadFn_t read;
  SensorGetRawFn_t get_raw;
  BarometerConvertFn_t convert;
} Barometer_t;

typedef struct {

} IMU_t;

typedef struct {

} Magnetometer_t;

typedef struct {

} Accelerometer_t;