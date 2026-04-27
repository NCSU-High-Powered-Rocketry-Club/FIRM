#include "sensor_task.h"

#include "mock_packet_handler_task.h"

#include "logger.h"
#include "mocking_handler.h"
#include <string.h>

static bool sensor_should_collect_mock(TaskCommandOption cmd_status, uint32_t notify_value) {
  return (cmd_status == TASKCMD_MOCK) && ((notify_value & SENSOR_NOTIFY_MOCK_BIT) != 0U);
}

static bool sensor_should_collect_live(TaskCommandOption cmd_status, uint32_t notify_value) {
  return (cmd_status != TASKCMD_MOCK) && ((notify_value & SENSOR_NOTIFY_ISR_BIT) != 0U);
}

static double sensor_cycles_to_seconds(uint32_t cycle_count) {
  uint32_t hclk_hz = HAL_RCC_GetHCLKFreq();
  if (hclk_hz == 0U) {
    return 0.0;
  }
  return (double)cycle_count / (double)hclk_hz;
}

static void sensor_notify_mock_handler_complete(void) {
  if (mock_packet_handler_handle != NULL) {
    xTaskNotifyGive(mock_packet_handler_handle);
  }
}

void collect_bmp581_data_task(void *argument) {
  (void)argument;

  const TickType_t max_wait = MAX_WAIT_TIME(BMP581_POLL_RATE_HZ);
  TaskCommandOption cmd_status = TASKCMD_LIVE;

  for (;;) {
    (void)xQueueReceive(bmp581_command_queue, &cmd_status, 0);

    uint32_t notify_value = 0U;
    if (xTaskNotifyWait(0U, 0xFFFFFFFFUL, &notify_value, max_wait) != pdTRUE) {
      continue;
    }

    bool do_mock = sensor_should_collect_mock(cmd_status, notify_value);
    bool do_live = sensor_should_collect_live(cmd_status, notify_value);
    if (!do_mock && !do_live) {
      continue;
    }

    BMP581RawData_t raw = {0};
    uint32_t cycle_count = 0U;

    if (do_mock) {
      cycle_count = mocking_handler_time_from_ring();
      if (mocking_handler_read_barometer(&raw) != 0) {
        sensor_notify_mock_handler_complete();
        continue;
      }
    } else {
      if (bmp581_read_data(&raw) != 0) {
        continue;
      }
      cycle_count = DWT->CYCCNT;
    }

    void *raw_log = logger_malloc_raw_storage(ID_BAROMETER, cycle_count);
    if (raw_log != NULL) {
      memcpy(raw_log, &raw, sizeof(raw));
    }

    BMP581BoardReading_t board = {0};
    bmp581_convert(&raw, &board);

    osMutexAcquire(sensorDataMutexHandle, osWaitForever);
    latest_data_packet.timestamp_seconds = sensor_cycles_to_seconds(cycle_count);
    latest_data_packet.temperature_celsius = board.temperature_celsius;
    latest_data_packet.pressure_pascals = board.pressure_pa;
    osMutexRelease(sensorDataMutexHandle);

    xEventGroupSetBits(sensors_collected, BMP581_TASK_BIT);

    if (do_mock) {
      sensor_notify_mock_handler_complete();
    }
  }
}

void collect_icm45686_data_task(void *argument) {
  (void)argument;

  const TickType_t max_wait = MAX_WAIT_TIME(ICM45686_POLL_RATE_HZ);
  TaskCommandOption cmd_status = TASKCMD_LIVE;

  for (;;) {
    (void)xQueueReceive(icm45686_command_queue, &cmd_status, 0);

    uint32_t notify_value = 0U;
    if (xTaskNotifyWait(0U, 0xFFFFFFFFUL, &notify_value, max_wait) != pdTRUE) {
      continue;
    }

    bool do_mock = sensor_should_collect_mock(cmd_status, notify_value);
    bool do_live = sensor_should_collect_live(cmd_status, notify_value);
    if (!do_mock && !do_live) {
      continue;
    }

    ICM45686RawData_t raw = {0};
    uint32_t cycle_count = 0U;

    if (do_mock) {
      cycle_count = mocking_handler_time_from_ring();
      if (mocking_handler_read_imu(&raw) != 0) {
        sensor_notify_mock_handler_complete();
        continue;
      }
    } else {
      if (icm45686_read_data(&raw) != 0) {
        continue;
      }
      cycle_count = DWT->CYCCNT;
    }

    void *raw_log = logger_malloc_raw_storage(ID_IMU, cycle_count);
    if (raw_log != NULL) {
      memcpy(raw_log, &raw, sizeof(raw));
    }

    ICM45686BoardReading_t board = {0};
    icm45686_convert_and_calibrate(&raw, &board);

    osMutexAcquire(sensorDataMutexHandle, osWaitForever);
    latest_data_packet.timestamp_seconds = sensor_cycles_to_seconds(cycle_count);
    latest_data_packet.raw_acceleration_x_gs = board.accel_x_g;
    latest_data_packet.raw_acceleration_y_gs = board.accel_y_g;
    latest_data_packet.raw_acceleration_z_gs = board.accel_z_g;
    latest_data_packet.raw_angular_rate_x_dps = board.gyro_x_dps;
    latest_data_packet.raw_angular_rate_y_dps = board.gyro_y_dps;
    latest_data_packet.raw_angular_rate_z_dps = board.gyro_z_dps;
    osMutexRelease(sensorDataMutexHandle);

    xEventGroupSetBits(sensors_collected, ICM45686_TASK_BIT);

    if (do_mock) {
      sensor_notify_mock_handler_complete();
    }
  }
}

void collect_mmc5983ma_data_task(void *argument) {
  (void)argument;

  const TickType_t max_wait = MAX_WAIT_TIME(MMC5983MA_POLL_RATE_HZ);
  TaskCommandOption cmd_status = TASKCMD_LIVE;

  for (;;) {
    (void)xQueueReceive(mmc5983ma_command_queue, &cmd_status, 0);

    uint32_t notify_value = 0U;
    if (xTaskNotifyWait(0U, 0xFFFFFFFFUL, &notify_value, max_wait) != pdTRUE) {
      continue;
    }

    bool do_mock = sensor_should_collect_mock(cmd_status, notify_value);
    bool do_live = sensor_should_collect_live(cmd_status, notify_value);
    if (!do_mock && !do_live) {
      continue;
    }

    MMC5983MARawData_t raw = {0};
    uint32_t cycle_count = 0U;

    if (do_mock) {
      cycle_count = mocking_handler_time_from_ring();
      if (mocking_handler_read_magnetometer(&raw) != 0) {
        sensor_notify_mock_handler_complete();
        continue;
      }
    } else {
      if (mmc5983ma_read_data(&raw) != 0) {
        continue;
      }
      cycle_count = DWT->CYCCNT;
    }

    void *raw_log = logger_malloc_raw_storage(ID_MAGNETOMETER, cycle_count);
    if (raw_log != NULL) {
      memcpy(raw_log, &raw, sizeof(raw));
    }

    MMC5983MABoardReading_t board = {0};
    mmc5983ma_convert_and_calibrate(&raw, &board);

    osMutexAcquire(sensorDataMutexHandle, osWaitForever);
    latest_data_packet.timestamp_seconds = sensor_cycles_to_seconds(cycle_count);
    latest_data_packet.magnetic_field_x_microteslas = board.magnetic_field_x_microteslas;
    latest_data_packet.magnetic_field_y_microteslas = board.magnetic_field_y_microteslas;
    latest_data_packet.magnetic_field_z_microteslas = board.magnetic_field_z_microteslas;
    osMutexRelease(sensorDataMutexHandle);

    xEventGroupSetBits(sensors_collected, MMC5983MA_TASK_BIT);

    if (do_mock) {
      sensor_notify_mock_handler_complete();
    }
  }
}

void collect_adxl371_data_task(void *argument) {
  (void)argument;

  const TickType_t max_wait = MAX_WAIT_TIME(ADXL371_POLL_RATE_HZ);
  TaskCommandOption cmd_status = TASKCMD_LIVE;

  for (;;) {
    (void)xQueueReceive(adxl371_command_queue, &cmd_status, 0);

    uint32_t notify_value = 0U;
    if (xTaskNotifyWait(0U, 0xFFFFFFFFUL, &notify_value, max_wait) != pdTRUE) {
      continue;
    }

    bool do_mock = sensor_should_collect_mock(cmd_status, notify_value);
    bool do_live = sensor_should_collect_live(cmd_status, notify_value);
    if (!do_mock && !do_live) {
      continue;
    }

    ADXL371RawData_t raw = {0};
    uint32_t cycle_count = 0U;

    if (do_mock) {
      cycle_count = mocking_handler_time_from_ring();
      if (mocking_handler_read_high_g(&raw) != 0) {
        sensor_notify_mock_handler_complete();
        continue;
      }
    } else {
      if (adxl371_read_data(&raw) != 0) {
        continue;
      }
      cycle_count = DWT->CYCCNT;
    }

    void *raw_log = logger_malloc_raw_storage(ID_HIGH_G_ACCELEROMETER, cycle_count);
    if (raw_log != NULL) {
      memcpy(raw_log, &raw, sizeof(raw));
    }

    ADXL371BoardReading_t board = {0};
    adxl371_convert_and_calibrate(&raw, &board);

    osMutexAcquire(sensorDataMutexHandle, osWaitForever);
    latest_data_packet.timestamp_seconds = sensor_cycles_to_seconds(cycle_count);
    latest_data_packet.high_g_accel_x_gs = board.accel_x_g;
    latest_data_packet.high_g_accel_y_gs = board.accel_y_g;
    latest_data_packet.high_g_accel_z_gs = board.accel_z_g;
    osMutexRelease(sensorDataMutexHandle);

    if (do_mock) {
      sensor_notify_mock_handler_complete();
    }
  }
}