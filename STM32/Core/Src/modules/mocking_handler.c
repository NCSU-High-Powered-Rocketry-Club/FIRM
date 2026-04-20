#include "mocking_handler.h"
#include "adxl371_packet.h"

static uint32_t mock_prev_ts = 0U;
static bool mock_have_prev_ts = false;

typedef struct {
  bool valid;
  MockPacketID id;
  uint32_t timestamp;
  union {
    BMP581RawData_t bmp581;
    ICM45686RawData_t icm45686;
    MMC5983MARawData_t mmc5983ma;
    ADXL371RawData_t adxl371;
  } raw;
} CachedMockInstance_t;

static CachedMockInstance_t cached_instance = {0};

static size_t mock_raw_size_for_id(const MockPacketID identifier) {
  switch (identifier) {
  case MOCKID_BMP581:
    return sizeof(BMP581RawData_t);
  case MOCKID_ICM45686:
    return sizeof(ICM45686RawData_t);
  case MOCKID_MMC5983MA:
    return sizeof(MMC5983MARawData_t);
  case MOCKID_ADXL371:
    return sizeof(ADXL371RawData_t);
  default:
    return 0U;
  }
}

static bool mock_adapter_pop_next_instance(void) {
  const uint8_t *peek = (const uint8_t *)mock_ring_peek();
  if (peek == NULL) {
    return false;
  }

  MockPacketID id = (MockPacketID)peek[0];
  size_t raw_size = mock_raw_size_for_id(id);
  if (raw_size == 0U) {
    return false;
  }

  size_t instance_size = 1U + sizeof(uint32_t) + raw_size;
  uint8_t *instance = (uint8_t *)mock_ring_pop(instance_size);
  if (instance == NULL) {
    return false;
  }

  cached_instance.id = id;
  memcpy(&cached_instance.timestamp, &instance[1], sizeof(uint32_t));
  switch (id) {
  case MOCKID_BMP581:
    memcpy(&cached_instance.raw.bmp581, &instance[1U + sizeof(uint32_t)], sizeof(BMP581RawData_t));
    break;
  case MOCKID_ICM45686:
    memcpy(&cached_instance.raw.icm45686, &instance[1U + sizeof(uint32_t)], sizeof(ICM45686RawData_t));
    break;
  case MOCKID_MMC5983MA:
    memcpy(&cached_instance.raw.mmc5983ma, &instance[1U + sizeof(uint32_t)], sizeof(MMC5983MARawData_t));
    break;
  case MOCKID_ADXL371:
    memcpy(&cached_instance.raw.adxl371, &instance[1U + sizeof(uint32_t)], sizeof(ADXL371RawData_t));
    break;
  default:
    return false;
  }

  cached_instance.valid = true;
  return true;
}

bool process_mock_settings_packet(uint8_t *received_bytes, uint32_t length,
                                  SystemSettings_t *settings,
                                  SensorScaleFactors_t *scale_factors) {
  const char *expected_header = FIRM_LOG_HEADER_TEXT;

  // Expected payload layout:
  // - "FIRM LOG v1.x\n" (14 bytes)
  // - SystemSettings struct
  // - HeaderFields struct

  size_t header_len = strlen(expected_header);

  // Verify header
  if (length < header_len || memcmp(received_bytes, expected_header, header_len) != 0) {
    return false;
  }

  // Extract SystemSettings
  uint32_t offset = (uint32_t)header_len;
  if (offset + sizeof(SystemSettings_t) > length) {
    return false;
  }
  memcpy(settings, &received_bytes[offset], sizeof(SystemSettings_t));
  offset += (uint32_t)sizeof(SystemSettings_t);

  // Extract HeaderFields
  if (offset + sizeof(SensorScaleFactors_t) > length) {
    return false;
  }
  
  // put the last of the header bytes into the scale factor fields
  memcpy(scale_factors, &received_bytes[offset], sizeof(SensorScaleFactors_t));
  if (settings_write_firm_settings(settings)) {
    return false;
  }
  return true;
}

bool mock_parse_sensor_packet(MockPacketID identifier, const uint8_t *payload_bytes,
                              uint32_t payload_len, SensorPacket *out_packet) {
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
  case MOCKID_ADXL371:
    if (sensor_len != (uint32_t)sizeof(ADXL371Packet_t)) {
      return false;
    }
    memcpy(&out_packet->packet.adxl371_packet, sensor_bytes, sizeof(ADXL371Packet_t));
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

uint32_t mock_timestamp_accumulate_delay_ms(uint32_t *accumulated_clock_cycles,
                                            const uint8_t timestamp_bytes[4]) {
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

bool mock_ring_push_sensor_instance(MockPacketID identifier,
                                    const uint8_t *payload_bytes,
                                    uint32_t payload_len) {
  const size_t raw_size = mock_raw_size_for_id(identifier);
  const size_t payload_header_size = sizeof(uint32_t);
  if (payload_bytes == NULL || raw_size == 0U) {
    return false;
  }

  if (payload_len != (uint32_t)(payload_header_size + raw_size)) {
    return false;
  }

  uint8_t packed_instance[1U + sizeof(uint32_t) + sizeof(ICM45686RawData_t)] = {0};
  const size_t instance_size = 1U + payload_header_size + raw_size;

  packed_instance[0] = (uint8_t)identifier;
  memcpy(&packed_instance[1], payload_bytes, payload_len);
  mock_ring_push(packed_instance, instance_size);
  return true;
}

uint32_t mock_adapter_get_time(void) {
  if (!cached_instance.valid) {
    (void)mock_adapter_pop_next_instance();
  }

  return cached_instance.valid ? cached_instance.timestamp : 0U;
}

int mock_adapter_read_bmp581(BMP581RawData_t *out) {
  if (!cached_instance.valid) {
    (void)mock_adapter_pop_next_instance();
  }

  if (!cached_instance.valid || cached_instance.id != MOCKID_BMP581 || out == NULL) {
    return 1;
  }

  *out = cached_instance.raw.bmp581;
  cached_instance.valid = false;
  return 0;
}

int mock_adapter_read_icm45686(ICM45686RawData_t *out) {
  if (!cached_instance.valid) {
    (void)mock_adapter_pop_next_instance();
  }

  if (!cached_instance.valid || cached_instance.id != MOCKID_ICM45686 || out == NULL) {
    return 1;
  }

  *out = cached_instance.raw.icm45686;
  cached_instance.valid = false;
  return 0;
}

int mock_adapter_read_mmc5983ma(MMC5983MARawData_t *out) {
  if (!cached_instance.valid) {
    (void)mock_adapter_pop_next_instance();
  }

  if (!cached_instance.valid || cached_instance.id != MOCKID_MMC5983MA || out == NULL) {
    return 1;
  }

  *out = cached_instance.raw.mmc5983ma;
  cached_instance.valid = false;
  return 0;
}

int mock_adapter_read_adxl371(ADXL371RawData_t *out) {
  if (!cached_instance.valid) {
    (void)mock_adapter_pop_next_instance();
  }

  if (!cached_instance.valid || cached_instance.id != MOCKID_ADXL371 || out == NULL) {
    return 1;
  }

  *out = cached_instance.raw.adxl371;
  cached_instance.valid = false;
  return 0;
}

void mock_adapter_reset_cached_instance(void) {
  cached_instance.valid = false;
  cached_instance.id = MOCKID_SETTINGS;
  cached_instance.timestamp = 0U;
}
