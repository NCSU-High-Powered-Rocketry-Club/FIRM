#include "mocking_handler.h"

static uint32_t mock_prev_ts = 0U;
static bool mock_have_prev_ts = false;

MockPacketID process_mock_packet(uint16_t identifier, uint32_t length, uint8_t *received_bytes, uint8_t *mock_packet) {
  // if its the settings, return immediately because it has to be processed differently
  if (identifier == MOCKID_SETTINGS) {
    return identifier;
  }
  memcpy(mock_packet, received_bytes, length);
  return identifier;
}

bool process_mock_settings_packet(uint8_t *received_bytes,
                                 uint32_t length,
                                 FIRMSettings_t* firm_settings,
                                 CalibrationSettings_t* calibration_settings,
                                 HeaderFields* header_fields,
                                 MockSettingsWriteFn write_fn,
                                 void *write_ctx) {
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
  uint32_t offset = (uint32_t)header_len;
  if (offset + sizeof(FIRMSettings_t) > length) {
    return false;
  }
  memcpy(firm_settings, &received_bytes[offset], sizeof(FIRMSettings_t));
  offset += (uint32_t)sizeof(FIRMSettings_t);

  // Extract CalibrationSettings
  if (offset + sizeof(CalibrationSettings_t) > length) {
    return false;
  }
  memcpy(calibration_settings, &received_bytes[offset], sizeof(CalibrationSettings_t));
  offset += (uint32_t)sizeof(CalibrationSettings_t);

  // Extract HeaderFields
  if (offset + sizeof(HeaderFields) > length) {
    return false;
  }
  memcpy(header_fields, &received_bytes[offset], sizeof(HeaderFields));

  // Delegate the side-effect (writing settings) to caller-provided callback.
  if (write_fn == NULL) {
    return false;
  }
  return write_fn(write_ctx, firm_settings, calibration_settings);
}

bool mock_parse_sensor_packet(MockPacketID identifier,
                             const uint8_t *payload_bytes,
                             uint32_t payload_len,
                             SensorPacket *out_packet) {
  if (payload_bytes == NULL || out_packet == NULL) {
    return false;
  }

  const uint32_t ts_len = (uint32_t)sizeof(out_packet->timestamp);
  if (payload_len < ts_len) {
    return false;
  }

  memcpy(out_packet->timestamp, payload_bytes, ts_len);

  const uint8_t *sensor_bytes = payload_bytes + ts_len;
  const uint32_t sensor_len = payload_len - ts_len;

  switch (identifier) {
    case MOCKID_BMP581:
      if (sensor_len != (uint32_t)sizeof(BMP581Packet_t)) {
        return false;
      }
      memcpy(&out_packet->packet.bmp581_packet, sensor_bytes, sizeof(BMP581Packet_t));
      return true;
    case MOCKID_ICM45686:
      if (sensor_len != (uint32_t)sizeof(ICM45686Packet_t)) {
        return false;
      }
      memcpy(&out_packet->packet.icm45686_packet, sensor_bytes, sizeof(ICM45686Packet_t));
      return true;
    case MOCKID_MMC5983MA:
      if (sensor_len != (uint32_t)sizeof(MMC5983MAPacket_t)) {
        return false;
      }
      memcpy(&out_packet->packet.mmc5983ma_packet, sensor_bytes, sizeof(MMC5983MAPacket_t));
      return true;
    case MOCKID_SETTINGS:
    default:
      return false;
  }
}

void mock_timestamp_accumulator_reset(void) {
  mock_have_prev_ts = false;
  mock_prev_ts = 0U;
}

uint32_t mock_timestamp_accumulate_delay_ms(uint32_t *accumulated_clock_cycles, const uint8_t timestamp_bytes[4]) {
  if (accumulated_clock_cycles == NULL || timestamp_bytes == NULL) {
    return 0U;
  }

  uint32_t ts = 0U;
  memcpy(&ts, timestamp_bytes, sizeof(ts));

  if (!mock_have_prev_ts) {
    mock_prev_ts = ts;
    mock_have_prev_ts = true;
    return 0U;
  }

  // Unsigned subtraction handles 32-bit wraparound.
  uint32_t delta_cycles = ts - mock_prev_ts;
  mock_prev_ts = ts;

  uint64_t acc = (uint64_t)(*accumulated_clock_cycles) + (uint64_t)delta_cycles;
  uint32_t delay_ms = (uint32_t)(acc / (uint64_t)MOCK_DEFAULT_CYCLES_PER_MS);
  uint32_t remainder = (uint32_t)(acc % (uint64_t)MOCK_DEFAULT_CYCLES_PER_MS);
  *accumulated_clock_cycles = remainder;
  return delay_ms;
}
