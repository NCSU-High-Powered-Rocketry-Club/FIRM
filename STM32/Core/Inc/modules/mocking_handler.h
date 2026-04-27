#pragma once

#include "adxl371_packet.h"
#include "bmp581_packet.h"
#include "data_processing/clock_cycle_count.h"
#include "data_processing/mocking_ring_buffer.h"
#include "icm45686_packet.h"
#include "mmc5983ma_packet.h"
#include "shared_data/identifiers.h"

#include <stdint.h>

typedef struct {
  void (*set_time_fn)(uint32_t (*time_fn)(void));
  void (*set_barometer_read_fn)(int (*read_fn)(BMP581RawData_t *));
  void (*set_imu_read_fn)(int (*read_fn)(ICM45686RawData_t *));
  void (*set_magnetometer_read_fn)(int (*read_fn)(MMC5983MARawData_t *));
  void (*set_high_g_read_fn)(int (*read_fn)(ADXL371RawData_t *));
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