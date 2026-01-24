#pragma once
#include <stdint.h>
#include <string.h>
#include "logger.h"
#include "settings.h"
#include <stdbool.h>

typedef enum {
  MOCKID_BMP581 = 'B',
  MOCKID_ICM45686 = 'I',
  MOCKID_MMC5983MA = 'M',
  MOCKID_SETTINGS = 'H',
} MockPacketID;

typedef bool (*MockSettingsWriteFn)(void *ctx,
                                   FIRMSettings_t *firm_settings,
                                   CalibrationSettings_t *calibration_settings);

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