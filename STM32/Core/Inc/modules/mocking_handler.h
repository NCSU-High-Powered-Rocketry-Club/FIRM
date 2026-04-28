#pragma once

#include "adxl371_packet.h"
#include "bmp581_packet.h"
#include "data_processing/clock_cycle_count.h"
#include "data_processing/mocking_ring_buffer.h"
#include "icm45686_packet.h"
#include "mmc5983ma_packet.h"
#include "shared_data/identifiers.h"

#include <stdbool.h>
#include <stdint.h>

typedef uint32_t (*MockTimeFn)(void);
typedef int (*MockBarometerReadFn)(BMP581RawData_t *);
typedef int (*MockImuReadFn)(ICM45686RawData_t *);
typedef int (*MockMagnetometerReadFn)(MMC5983MARawData_t *);
typedef int (*MockHighGReadFn)(ADXL371RawData_t *);

typedef struct {
  void (*set_time_fn)(MockTimeFn time_fn);
  void (*set_barometer_read_fn)(MockBarometerReadFn read_fn);
  void (*set_imu_read_fn)(MockImuReadFn read_fn);
  void (*set_magnetometer_read_fn)(MockMagnetometerReadFn read_fn);
  void (*set_high_g_read_fn)(MockHighGReadFn read_fn);
  MockTimeFn (*get_time_fn)(void);
  MockBarometerReadFn (*get_barometer_read_fn)(void);
  MockImuReadFn (*get_imu_read_fn)(void);
  MockMagnetometerReadFn (*get_magnetometer_read_fn)(void);
  MockHighGReadFn (*get_high_g_read_fn)(void);
} MockSensorTaskInjectFns_t;

/**
 * @brief Initializes mock dispatch internals and resets ring/counter state.
 */
void mocking_handler_init(const MockRingCountSemaphore_t *counting_semaphore,
                          uint32_t clock_speed_mhz);

/**
 * @brief Resets the delay-timestamp state used by dispatch_mock_msg.
 */
void mocking_handler_reset_delay_state(void);

/**
 * @brief Stores optional setter callbacks used to inject mock fns into sensor task/module.
 */
void mocking_handler_configure_sensor_task_injection(const MockSensorTaskInjectFns_t *inject_fns);

/**
 * @brief Applies previously configured injection callbacks.
 * @note Safe to call even if some callbacks are NULL.
 */
void mocking_handler_inject_sensor_task_hooks(void);

/**
 * @brief Starts mock mode by injecting mock sensor callbacks.
 * @return true if mock mode started; false otherwise.
 */
bool mocking_handler_start_mock(void);

/**
 * @brief Cancels mock mode and restores live sensor callbacks.
 * @return true if mock mode was cancelled; false otherwise.
 */
bool mocking_handler_cancel_mock(void);

/**
 * @brief Dispatches one mock message into the mock ring.
 *
 * Message format expected:
 *   [identifier (1)][timestamp_cyccnt (4)][sensor raw payload]
 *
 * @return Recommended delay in milliseconds relative to previous mock timestamp.
 */
uint32_t dispatch_mock_msg(const uint8_t *mock_message);

/**
 * @brief Time callback intended for sensor task injection in mock mode.
 */
uint32_t mocking_handler_time_from_ring(void);

int mocking_handler_read_barometer(BMP581RawData_t *out);
int mocking_handler_read_imu(ICM45686RawData_t *out);
int mocking_handler_read_magnetometer(MMC5983MARawData_t *out);
int mocking_handler_read_high_g(ADXL371RawData_t *out);