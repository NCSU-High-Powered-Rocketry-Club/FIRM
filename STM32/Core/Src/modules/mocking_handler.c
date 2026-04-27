#include "mocking_handler.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#ifndef MOCK_CLOCK_DEFAULT_MHZ
#define MOCK_CLOCK_DEFAULT_MHZ 168U
#endif

#define MOCK_TS_OFFSET_BYTES 1U
#define MOCK_HEADER_BYTES (1U + sizeof(uint32_t))

static ClockCycleCounter_t mock_delay_counter = {
    .dwt_overflow_count = 0U,
    .last_cyccnt = 0U,
    .clock_speed_hz = MOCK_CLOCK_DEFAULT_MHZ * 1000000U,
};

static bool mock_delay_has_prev = false;
static double mock_prev_delay_seconds = 0.0;
static bool mock_ring_ready = false;
static uint32_t mock_last_timestamp_cycles = 0U;
static MockSensorTaskInjectFns_t mock_sensor_inject_fns = {0};

static size_t mock_raw_size_for_id(Identifiers_t id) {
  switch (id) {
  case ID_BAROMETER:
    return sizeof(BMP581RawData_t);
  case ID_IMU:
    return sizeof(ICM45686RawData_t);
  case ID_MAGNETOMETER:
    return sizeof(MMC5983MARawData_t);
  case ID_HIGH_G_ACCELEROMETER:
    return sizeof(ADXL371RawData_t);
  default:
    return 0U;
  }
}

static size_t mock_instance_size_for_id(Identifiers_t id) {
  return MOCK_HEADER_BYTES + mock_raw_size_for_id(id);
}

static uint32_t mock_delay_ms_from_timestamp(uint32_t timestamp_cycles) {
  uint32_t delay_ms = 0U;
  double now_seconds = clock_cycle_counter_process(&mock_delay_counter, timestamp_cycles);

  if (mock_delay_has_prev) {
    double delta_seconds = now_seconds - mock_prev_delay_seconds;
    if (delta_seconds > 0.0) {
      double delta_ms = delta_seconds * 1000.0;
      if (delta_ms >= (double)UINT32_MAX) {
        delay_ms = UINT32_MAX;
      } else {
        delay_ms = (uint32_t)delta_ms;
      }
    }
  }

  mock_prev_delay_seconds = now_seconds;
  mock_delay_has_prev = true;
  return delay_ms;
}

static bool mock_pop_sensor_payload(Identifiers_t expected_id, void *out_payload,
                                    size_t payload_size) {
  if (out_payload == NULL || payload_size == 0U) {
    return false;
  }

  const uint8_t *peek = (const uint8_t *)mock_ring_peek();
  if (peek == NULL || (Identifiers_t)peek[0] != expected_id) {
    return false;
  }

  const size_t instance_size = mock_instance_size_for_id(expected_id);
  if (instance_size == 0U) {
    return false;
  }

  uint8_t *instance = (uint8_t *)mock_ring_pop(instance_size);
  if (instance == NULL) {
    return false;
  }

  if ((Identifiers_t)instance[0] != expected_id) {
    return false;
  }

  memcpy(&mock_last_timestamp_cycles, &instance[MOCK_TS_OFFSET_BYTES], sizeof(uint32_t));
  memcpy(out_payload, &instance[MOCK_HEADER_BYTES], payload_size);
  return true;
}

void mocking_handler_init(const MockRingCountSemaphore_t *counting_semaphore,
                          uint32_t clock_speed_mhz) {
  if (counting_semaphore == NULL) {
    mock_ring_ready = false;
    return;
  }

  mock_ring_setup(counting_semaphore);
  mock_ring_ready = true;

  if (clock_speed_mhz == 0U) {
    clock_speed_mhz = MOCK_CLOCK_DEFAULT_MHZ;
  }

  clock_cycle_counter_init(&mock_delay_counter, clock_speed_mhz);
  mocking_handler_reset_delay_state();
}

void mocking_handler_reset_delay_state(void) {
  clock_cycle_counter_reset(&mock_delay_counter);
  mock_delay_has_prev = false;
  mock_prev_delay_seconds = 0.0;
  mock_last_timestamp_cycles = 0U;
}

void mocking_handler_configure_sensor_task_injection(const MockSensorTaskInjectFns_t *inject_fns) {
  if (inject_fns == NULL) {
    memset(&mock_sensor_inject_fns, 0, sizeof(mock_sensor_inject_fns));
    return;
  }

  mock_sensor_inject_fns = *inject_fns;
}

void mocking_handler_inject_sensor_task_hooks(void) {
  if (mock_sensor_inject_fns.set_time_fn != NULL) {
    mock_sensor_inject_fns.set_time_fn(mocking_handler_time_from_ring);
  }
  if (mock_sensor_inject_fns.set_barometer_read_fn != NULL) {
    mock_sensor_inject_fns.set_barometer_read_fn(mocking_handler_read_barometer);
  }
  if (mock_sensor_inject_fns.set_imu_read_fn != NULL) {
    mock_sensor_inject_fns.set_imu_read_fn(mocking_handler_read_imu);
  }
  if (mock_sensor_inject_fns.set_magnetometer_read_fn != NULL) {
    mock_sensor_inject_fns.set_magnetometer_read_fn(mocking_handler_read_magnetometer);
  }
  if (mock_sensor_inject_fns.set_high_g_read_fn != NULL) {
    mock_sensor_inject_fns.set_high_g_read_fn(mocking_handler_read_high_g);
  }
}

uint32_t dispatch_mock_msg(const uint8_t *mock_message) {
  if (!mock_ring_ready || mock_message == NULL) {
    return 0U;
  }

  const Identifiers_t id = (Identifiers_t)mock_message[0];
  const size_t instance_size = mock_instance_size_for_id(id);
  if (instance_size == 0U) {
    return 0U;
  }

  uint32_t timestamp_cycles = 0U;
  memcpy(&timestamp_cycles, &mock_message[MOCK_TS_OFFSET_BYTES], sizeof(timestamp_cycles));

  mock_ring_push((uint8_t *)mock_message, instance_size);
  return mock_delay_ms_from_timestamp(timestamp_cycles);
}

uint32_t mocking_handler_time_from_ring(void) {
  const uint8_t *peek = (const uint8_t *)mock_ring_peek();
  if (peek == NULL) {
    return mock_last_timestamp_cycles;
  }

  uint32_t timestamp_cycles = 0U;
  memcpy(&timestamp_cycles, &peek[MOCK_TS_OFFSET_BYTES], sizeof(timestamp_cycles));
  return timestamp_cycles;
}

int mocking_handler_read_barometer(BMP581RawData_t *out) {
  return mock_pop_sensor_payload(ID_BAROMETER, out, sizeof(BMP581RawData_t)) ? 0 : 1;
}

int mocking_handler_read_imu(ICM45686RawData_t *out) {
  return mock_pop_sensor_payload(ID_IMU, out, sizeof(ICM45686RawData_t)) ? 0 : 1;
}

int mocking_handler_read_magnetometer(MMC5983MARawData_t *out) {
  return mock_pop_sensor_payload(ID_MAGNETOMETER, out, sizeof(MMC5983MARawData_t)) ? 0 : 1;
}

int mocking_handler_read_high_g(ADXL371RawData_t *out) {
  return mock_pop_sensor_payload(ID_HIGH_G_ACCELEROMETER, out, sizeof(ADXL371RawData_t)) ? 0 : 1;
}
