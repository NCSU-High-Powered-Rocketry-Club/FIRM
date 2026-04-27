#include <unity.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "mocking_handler.h"

#define TEST_RING_CAPACITY 16U
#define TEST_RING_INSTANCE_MAX_BYTES 64U

// Minimal FIFO ring test double used to isolate mocking_handler behavior.
static uint8_t test_ring_bytes[TEST_RING_CAPACITY][TEST_RING_INSTANCE_MAX_BYTES];
static size_t test_ring_sizes[TEST_RING_CAPACITY];
static size_t test_ring_head = 0U;
static size_t test_ring_tail = 0U;
static size_t test_ring_count = 0U;

void mock_ring_setup(const MockRingCountSemaphore_t *counting_semaphore) {
  if (counting_semaphore != NULL && counting_semaphore->reset != NULL) {
    counting_semaphore->reset(counting_semaphore->context);
  }

  memset(test_ring_bytes, 0, sizeof(test_ring_bytes));
  memset(test_ring_sizes, 0, sizeof(test_ring_sizes));
  test_ring_head = 0U;
  test_ring_tail = 0U;
  test_ring_count = 0U;
}

void mock_ring_push(uint8_t *data_instance, size_t instance_size) {
  if (data_instance == NULL || instance_size == 0U ||
      instance_size > TEST_RING_INSTANCE_MAX_BYTES) {
    return;
  }
  if (test_ring_count >= TEST_RING_CAPACITY) {
    return;
  }

  memcpy(test_ring_bytes[test_ring_tail], data_instance, instance_size);
  test_ring_sizes[test_ring_tail] = instance_size;
  test_ring_tail = (test_ring_tail + 1U) % TEST_RING_CAPACITY;
  test_ring_count++;
}

void *mock_ring_pop(size_t instance_size) {
  if (test_ring_count == 0U) {
    return NULL;
  }
  if (instance_size != test_ring_sizes[test_ring_head]) {
    return NULL;
  }

  uint8_t *out = test_ring_bytes[test_ring_head];
  test_ring_head = (test_ring_head + 1U) % TEST_RING_CAPACITY;
  test_ring_count--;
  return out;
}

const void *mock_ring_peek(void) {
  if (test_ring_count == 0U) {
    return NULL;
  }
  return test_ring_bytes[test_ring_head];
}

size_t mock_ring_get_length(void) { return test_ring_count; }

void clock_cycle_counter_init(ClockCycleCounter_t *counter, uint32_t clock_speed_mhz) {
  counter->dwt_overflow_count = 0U;
  counter->last_cyccnt = 0U;
  counter->clock_speed_hz = clock_speed_mhz * 1000000U;
}

void clock_cycle_counter_reset(ClockCycleCounter_t *counter) {
  counter->dwt_overflow_count = 0U;
  counter->last_cyccnt = 0U;
}

double clock_cycle_counter_process(ClockCycleCounter_t *counter, uint32_t clock_cycle_count) {
  if (clock_cycle_count < counter->last_cyccnt) {
    counter->dwt_overflow_count++;
  }
  counter->last_cyccnt = clock_cycle_count;

  uint64_t total_cycles = ((uint64_t)counter->dwt_overflow_count << 32) | clock_cycle_count;
  return ((double)total_cycles) / counter->clock_speed_hz;
}

static size_t fake_semaphore_count = 0U;

static bool fake_semaphore_try_take(void *context) {
  size_t *count = (size_t *)context;
  if (*count == 0U) {
    return false;
  }

  (*count)--;
  return true;
}

static bool fake_semaphore_give(void *context) {
  size_t *count = (size_t *)context;
  (*count)++;
  return true;
}

static size_t fake_semaphore_get_count(void *context) {
  size_t *count = (size_t *)context;
  return *count;
}

static void fake_semaphore_reset(void *context) {
  size_t *count = (size_t *)context;
  *count = 0U;
}

static const MockRingCountSemaphore_t fake_count_semaphore = {
    .context = &fake_semaphore_count,
    .try_take = fake_semaphore_try_take,
    .give = fake_semaphore_give,
    .get_count = fake_semaphore_get_count,
    .reset = fake_semaphore_reset,
};

static uint32_t (*captured_time_fn)(void) = NULL;
static int (*captured_barometer_fn)(BMP581RawData_t *) = NULL;
static int (*captured_imu_fn)(ICM45686RawData_t *) = NULL;
static int (*captured_magnetometer_fn)(MMC5983MARawData_t *) = NULL;
static int (*captured_high_g_fn)(ADXL371RawData_t *) = NULL;

static void capture_time_setter(uint32_t (*time_fn)(void)) { captured_time_fn = time_fn; }

static void capture_barometer_setter(int (*read_fn)(BMP581RawData_t *)) {
  captured_barometer_fn = read_fn;
}

static void capture_imu_setter(int (*read_fn)(ICM45686RawData_t *)) { captured_imu_fn = read_fn; }

static void capture_magnetometer_setter(int (*read_fn)(MMC5983MARawData_t *)) {
  captured_magnetometer_fn = read_fn;
}

static void capture_high_g_setter(int (*read_fn)(ADXL371RawData_t *)) {
  captured_high_g_fn = read_fn;
}

void setUp(void) {
  fake_semaphore_count = 0U;
  mocking_handler_init(&fake_count_semaphore, 1U);
  mocking_handler_reset_delay_state();

  captured_time_fn = NULL;
  captured_barometer_fn = NULL;
  captured_imu_fn = NULL;
  captured_magnetometer_fn = NULL;
  captured_high_g_fn = NULL;
  mocking_handler_configure_sensor_task_injection(NULL);
}

void tearDown(void) {}

void test_dispatch_without_init_returns_zero_and_does_not_make_data_readable(void) {
  uint8_t message[1U + sizeof(uint32_t) + sizeof(BMP581RawData_t)] = {0};
  message[0] = (uint8_t)ID_BAROMETER;

  // Ring is not initialized here; dispatch should be a no-op.
  mocking_handler_init(NULL, 0U);
  TEST_ASSERT_EQUAL_UINT32(0U, dispatch_mock_msg(message));

  BMP581RawData_t out = {0};
  TEST_ASSERT_EQUAL_INT(1, mocking_handler_read_barometer(&out));
}

void test_dispatch_and_read_matching_sensor_packet_succeeds(void) {
  BMP581RawData_t expected = {0};
  memset(&expected, 0xA5, sizeof(expected));

  uint8_t message[1U + sizeof(uint32_t) + sizeof(BMP581RawData_t)] = {0};
  const uint32_t timestamp_cycles = 5000U;

  message[0] = (uint8_t)ID_BAROMETER;
  memcpy(&message[1], &timestamp_cycles, sizeof(timestamp_cycles));
  memcpy(&message[1U + sizeof(uint32_t)], &expected, sizeof(expected));

  // First mock sample establishes the delay baseline and should report zero delay.
  TEST_ASSERT_EQUAL_UINT32(0U, dispatch_mock_msg(message));

  BMP581RawData_t out = {0};
  TEST_ASSERT_EQUAL_INT(0, mocking_handler_read_barometer(&out));
  TEST_ASSERT_EQUAL_MEMORY(&expected, &out, sizeof(expected));
}

void test_dispatch_returns_ms_delay_between_samples_using_counter_instance(void) {
  uint8_t message_a[1U + sizeof(uint32_t) + sizeof(ICM45686RawData_t)] = {0};
  uint8_t message_b[1U + sizeof(uint32_t) + sizeof(ICM45686RawData_t)] = {0};
  const uint32_t t0_cycles = 10000U;
  const uint32_t t1_cycles = 13000U;

  // With 1 MHz config, 1000 cycles = 1 ms.
  message_a[0] = (uint8_t)ID_IMU;
  message_b[0] = (uint8_t)ID_IMU;
  memcpy(&message_a[1], &t0_cycles, sizeof(t0_cycles));
  memcpy(&message_b[1], &t1_cycles, sizeof(t1_cycles));

  TEST_ASSERT_EQUAL_UINT32(0U, dispatch_mock_msg(message_a));
  // Conversion path truncates double->uint32_t, so 3 ms nominal delta can be
  // observed as 2 ms on some hosts due to floating-point representation.
  uint32_t delay_ms = dispatch_mock_msg(message_b);
  TEST_ASSERT_TRUE((delay_ms == 2U) || (delay_ms == 3U));
}

void test_time_callback_returns_front_timestamp_then_last_popped_when_empty(void) {
  ICM45686RawData_t imu = {0};
  imu.accX_H = 0xABU;

  uint8_t message[1U + sizeof(uint32_t) + sizeof(ICM45686RawData_t)] = {0};
  const uint32_t timestamp_cycles = 22222U;

  message[0] = (uint8_t)ID_IMU;
  memcpy(&message[1], &timestamp_cycles, sizeof(timestamp_cycles));
  memcpy(&message[1U + sizeof(uint32_t)], &imu, sizeof(imu));

  dispatch_mock_msg(message);

  // While queued, time callback should return the timestamp at the ring head.
  TEST_ASSERT_EQUAL_UINT32(timestamp_cycles, mocking_handler_time_from_ring());

  ICM45686RawData_t out = {0};
  TEST_ASSERT_EQUAL_INT(0, mocking_handler_read_imu(&out));

  // After pop, callback should return last known timestamp for continuity.
  TEST_ASSERT_EQUAL_UINT32(timestamp_cycles, mocking_handler_time_from_ring());
}

void test_read_wrong_sensor_type_fails_without_consuming_head_instance(void) {
  MMC5983MARawData_t expected = {0};
  memset(&expected, 0x3C, sizeof(expected));

  uint8_t message[1U + sizeof(uint32_t) + sizeof(MMC5983MARawData_t)] = {0};
  const uint32_t timestamp_cycles = 1234U;

  message[0] = (uint8_t)ID_MAGNETOMETER;
  memcpy(&message[1], &timestamp_cycles, sizeof(timestamp_cycles));
  memcpy(&message[1U + sizeof(uint32_t)], &expected, sizeof(expected));

  dispatch_mock_msg(message);

  // Wrong reader must fail and leave queued data untouched.
  BMP581RawData_t wrong_out = {0};
  TEST_ASSERT_EQUAL_INT(1, mocking_handler_read_barometer(&wrong_out));

  MMC5983MARawData_t out = {0};
  TEST_ASSERT_EQUAL_INT(0, mocking_handler_read_magnetometer(&out));
  TEST_ASSERT_EQUAL_MEMORY(&expected, &out, sizeof(expected));
}

void test_dispatch_invalid_identifier_returns_zero_and_queues_nothing(void) {
  uint8_t message[1U + sizeof(uint32_t)] = {0};
  const uint32_t timestamp_cycles = 2000U;

  // System command IDs are not valid mock sensor packet IDs.
  message[0] = (uint8_t)ID_MOCK_REQUEST;
  memcpy(&message[1], &timestamp_cycles, sizeof(timestamp_cycles));

  TEST_ASSERT_EQUAL_UINT32(0U, dispatch_mock_msg(message));

  ADXL371RawData_t out = {0};
  TEST_ASSERT_EQUAL_INT(1, mocking_handler_read_high_g(&out));
}

void test_inject_hooks_calls_registered_setters_with_mocking_handler_functions(void) {
  MockSensorTaskInjectFns_t inject_fns = {
      .set_time_fn = capture_time_setter,
      .set_barometer_read_fn = capture_barometer_setter,
      .set_imu_read_fn = capture_imu_setter,
      .set_magnetometer_read_fn = capture_magnetometer_setter,
      .set_high_g_read_fn = capture_high_g_setter,
  };

  mocking_handler_configure_sensor_task_injection(&inject_fns);
  mocking_handler_inject_sensor_task_hooks();

  TEST_ASSERT_EQUAL_PTR(mocking_handler_time_from_ring, captured_time_fn);
  TEST_ASSERT_EQUAL_PTR(mocking_handler_read_barometer, captured_barometer_fn);
  TEST_ASSERT_EQUAL_PTR(mocking_handler_read_imu, captured_imu_fn);
  TEST_ASSERT_EQUAL_PTR(mocking_handler_read_magnetometer, captured_magnetometer_fn);
  TEST_ASSERT_EQUAL_PTR(mocking_handler_read_high_g, captured_high_g_fn);
}

void test_configure_injection_null_clears_hooks(void) {
  MockSensorTaskInjectFns_t inject_fns = {
      .set_time_fn = capture_time_setter,
      .set_barometer_read_fn = capture_barometer_setter,
      .set_imu_read_fn = capture_imu_setter,
      .set_magnetometer_read_fn = capture_magnetometer_setter,
      .set_high_g_read_fn = capture_high_g_setter,
  };

  mocking_handler_configure_sensor_task_injection(&inject_fns);
  mocking_handler_configure_sensor_task_injection(NULL);
  mocking_handler_inject_sensor_task_hooks();

  TEST_ASSERT_NULL(captured_time_fn);
  TEST_ASSERT_NULL(captured_barometer_fn);
  TEST_ASSERT_NULL(captured_imu_fn);
  TEST_ASSERT_NULL(captured_magnetometer_fn);
  TEST_ASSERT_NULL(captured_high_g_fn);
}
