#pragma once
#include <stdint.h>
#include <string.h>
#include "settings.h"
#include <stdbool.h>
#include "data_preprocess.h"

// HeaderFields is owned by logger.h. In host unit tests we avoid including
// logger.h so Ceedling doesn't pull in logger.c (which depends on FATFS).
#ifndef TEST
  #include "logger.h"
#else
  typedef struct {
      float temp_sf;
      float pressure_sf;
      float accel_sf;
      float angular_rate_sf;
      float magnetic_field_sf;
  } HeaderFields;
#endif

typedef enum {
  MOCKID_BMP581 = 'B',
  MOCKID_ICM45686 = 'I',
  MOCKID_MMC5983MA = 'M',
  MOCKID_SETTINGS = 'H',
} MockPacketID;

/**
 * Builds a SensorPacket from a mock sensor payload.
 *
 * Payload layout:
 *   [timestamp (4)][sensor payload bytes]
 *
 * @param identifier One of 'B','I','M'
 * @param payload_bytes Pointer to payload bytes (no CRC)
 * @param payload_len Length of payload bytes
 * @param out_packet Output SensorPacket
 * @return true on success, false on invalid identifier/length
 */
bool mock_parse_sensor_packet(MockPacketID identifier,
                             const uint8_t *payload_bytes,
                             uint32_t payload_len,
                             SensorPacket *out_packet);

// ----------------------------
// Mock timestamp scheduling
// ----------------------------
// Mock sensor packets carry a 32-bit DWT cycle counter timestamp (little-endian).
// These helpers accumulate cycle deltas between packets and return how many
// whole milliseconds the caller should delay for.
//
// Notes:
// - First packet after reset returns 0ms (no multi-second initial delay).
// - Handles 32-bit wraparound using unsigned subtraction.
// - Pure logic: no HAL/RTOS dependencies.

#define MOCK_DEFAULT_CYCLES_PER_MS (168000U)

// Resets internal "previous timestamp" state.
void mock_timestamp_accumulator_reset(void);

// Accumulates delta cycles into *accumulated_clock_cycles and returns delay_ms.
// Uses MOCK_DEFAULT_CYCLES_PER_MS.
uint32_t mock_timestamp_accumulate_delay_ms(uint32_t *accumulated_clock_cycles, const uint8_t timestamp_bytes[4]);

typedef bool (*MockSettingsWriteFn)(void *ctx, FIRMSettings_t *firm_settings, CalibrationSettings_t *calibration_settings);

MockPacketID process_mock_packet(uint16_t identifier, uint32_t length, uint8_t *received_bytes, uint8_t *mock_packet);

/**
 * Processes a mock header/settings packet by parsing the settings and calibration data
 * and writing them to sector 2 of the flash chip.
 *
 * @param received_bytes pointer to the received payload bytes
 * @param length length of the payload
 * @param firm_settings pointer to FIRMSettings_t struct to be filled from the payload
 * @param calibration_settings pointer to CalibrationSettings_t struct to be filled from the payload
 * @param header_fields pointer to HeaderFields struct to be filled from the payload
 * @return true on success, false on failure
 */
bool process_mock_settings_packet(uint8_t *received_bytes,
                                 uint32_t length,
                                 FIRMSettings_t* firm_settings,
                                 CalibrationSettings_t* calibration_settings,
                                 HeaderFields* header_fields,
                                 MockSettingsWriteFn write_fn,
                                 void *write_ctx);
