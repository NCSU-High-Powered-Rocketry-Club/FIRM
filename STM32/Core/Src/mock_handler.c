#include "mock_handler.h"
#include "led.h"
#include "settings.h"
#include "logger.h"

MockPacketID process_mock_packet(uint16_t identifier, uint32_t length, uint8_t *received_bytes, uint8_t *mock_packet) {
  // if its the settings, return immediately because it has to be processed differently
  if (identifier == MOCKID_SETTINGS)
    return identifier;
  memcpy(mock_packet, received_bytes, length);
  return identifier;
}

bool process_mock_settings_packet(uint8_t *received_bytes, uint32_t length, FIRMSettings_t* firm_settings, CalibrationSettings_t* calibration_settings, HeaderFields* header_fields) {
  const char* expected_header = "FIRM LOG v1.1\n";

  // Expected payload layout:
  // - "FIRM LOG v1.1\n" (14 bytes)
  // - FirmSettings struct
  // - CalibrationSettings struct
  // - HeaderFields struct
  
  
  size_t header_len = strlen(expected_header);
  
  // Verify header
  if (length < header_len || memcmp(received_bytes, expected_header, header_len) != 0) {
    return false;
  }
  
  // Extract FirmSettings
  uint32_t offset = header_len;
  if (offset + sizeof(FIRMSettings_t) > length) {
    return false;
  }
  memcpy(firm_settings, &received_bytes[offset], sizeof(FIRMSettings_t));
  offset += sizeof(FIRMSettings_t);
  
  // Extract CalibrationSettings
  if (offset + sizeof(CalibrationSettings_t) > length) {
    return false;
  }
  memcpy(calibration_settings, &received_bytes[offset], sizeof(CalibrationSettings_t));
  offset += sizeof(CalibrationSettings_t);
  
  // Extract HeaderFields
  if (offset + sizeof(HeaderFields) > length) {
    return false;
  }
  memcpy(header_fields, &received_bytes[offset], sizeof(HeaderFields));
  
  // Write the mock settings to sector 2
  bool success = settings_write_mock_settings(firm_settings, calibration_settings);
  return success;
}