#include "sensor_manager.h"

static uint32_t (*get_time)(void);
static ClockCycleCounter_t clock_counter = {
    .dwt_overflow_count = 0U,
    .last_cyccnt = 0U,
    .clock_speed_hz = 168000000U,
};

void set_time_fn(uint32_t (*time_fn)(void)) { get_time = time_fn; }

void sensor_collect_data(Identifiers_t sensor, DataPacket_t *board_readings) {
  uint32_t clock_cycles = (get_time != NULL) ? get_time() : 0U;
  // allocate bytes in the logger for the raw sensor data
  void *raw_data_storage = logger_malloc_raw_storage(sensor, clock_cycles);

  // the process for each sensor is the same:
  // 1. read data into the raw data storage defined above
  // 2. convert the raw data to board-frame floats, output directly into the global struct
  switch (sensor) {
  case ID_BAROMETER:
    bmp581_read_data((BMP581RawData_t *)raw_data_storage);
    bmp581_convert(raw_data_storage,
                   (BMP581BoardReading_t *)(&board_readings->temperature_celsius));
    break;
  case ID_IMU:
    icm45686_read_data((ICM45686RawData_t *)raw_data_storage);
    icm45686_convert_and_calibrate(
        raw_data_storage, (ICM45686BoardReading_t *)(&board_readings->raw_acceleration_x_gs));
    break;
  case ID_MAGNETOMETER:
    mmc5983ma_read_data((MMC5983MARawData_t *)raw_data_storage);
    mmc5983ma_convert_and_calibrate(
        raw_data_storage,
        (MMC5983MABoardReading_t *)(&board_readings->magnetic_field_x_microteslas));
    break;
  case ID_HIGH_G_ACCELEROMETER:
    adxl371_read_data((ADXL371RawData_t *)raw_data_storage);
    adxl371_convert_and_calibrate(raw_data_storage,
                                  (ADXL371BoardReading_t *)(&board_readings->high_g_accel_x_gs));
  }

  board_readings->timestamp_seconds = clock_cycle_counter_process(&clock_counter, clock_cycles);
}
