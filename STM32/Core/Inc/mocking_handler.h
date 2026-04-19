#pragma once
#include <stdint.h>
#include <string.h>
#include "settings_manager.h"
#include "sensors.h"
#include "bmp581_packet.h"
#include "icm45686_packet.h"
#include "mmc5983ma_packet.h"
#include "adxl371_packet.h"
#include "data_processing/mocking_ring_buffer.h"
#include <stdbool.h>
#include "data_preprocess.h"
#include "system_settings.h"


typedef enum {
  MOCKID_BMP581 = 'B',
  MOCKID_ICM45686 = 'I',
  MOCKID_MMC5983MA = 'M',
  MOCKID_ADXL371 = 'A',
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
                                 SystemSettings_t* settings,
                                 SensorScaleFactors_t* header_fields);

/**
 * @brief Pushes a parsed mock sensor payload into the mock ring in adapter format.
 *
 * @param identifier Mock sensor identifier.
 * @param payload_bytes Payload bytes: [timestamp(4)][raw sensor bytes].
 * @param payload_len Number of payload bytes.
 * @retval true on successful enqueue, false on invalid payload.
 */
bool mock_ring_push_sensor_instance(MockPacketID identifier,
                                    const uint8_t *payload_bytes,
                                    uint32_t payload_len);

/**
 * @brief Returns timestamp associated with the currently cached mock instance.
 * @note Intended for sensor_manager's injected time callback.
 */
uint32_t mock_adapter_get_time(void);

/**
 * @brief Injected mock read adapters for sensor_manager callbacks.
 */
int mock_adapter_read_bmp581(BMP581RawData_t *out);
int mock_adapter_read_icm45686(ICM45686RawData_t *out);
int mock_adapter_read_mmc5983ma(MMC5983MARawData_t *out);
int mock_adapter_read_adxl371(ADXL371RawData_t *out);

/**
 * @brief Clears cached mock adapter instance state.
 */
void mock_adapter_reset_cached_instance(void);
