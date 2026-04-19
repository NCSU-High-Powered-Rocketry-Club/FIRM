#include "sensor_manager.h"


// function pointers for collecting data. Can be substituted when in mock mode
static int (*barometer_read_data)(BMP581RawData_t *) = bmp581_read_data;
static int (*imu_read_data)(ICM45686RawData_t *) = icm45686_read_data;
static int (*magnetometer_read_data)(MMC5983MARawData_t *) = mmc5983ma_read_data;
static int (*high_g_read_data)(ADXL371RawData_t *) = adxl371_read_data;

static uint32_t (*get_time)(void);

void set_time_fn(uint32_t(*time_fn)(void)) {
  get_time = time_fn;
}

void sensor_collect_data(Sensors_t sensor, DataPacket *board_readings) {
  uint32_t clock_cycles = get_time();
  // allocate bytes in the logger for the raw sensor data
  void *raw_data_storage = logger_malloc_raw_storage(sensor, clock_cycles);

  // the process for each sensor is the same:
  // 1. read data into the raw data storage defined above
  // 2. convert the raw data to board-frame floats, output directly into the global struct
  switch (sensor) {
    case BAROMETER:
      barometer_read_data((BMP581RawData_t *)raw_data_storage);
      bmp581_convert(raw_data_storage, (BMP581BoardReading_t *)(&board_readings->temperature_celsius));
      break;
    case IMU:
      imu_read_data((ICM45686RawData_t *)raw_data_storage);
      icm45686_convert_and_calibrate(raw_data_storage, (ICM45686BoardReading_t *)(&board_readings->raw_acceleration_x_gs));
      break;
    case MAGNETOMETER:
      magnetometer_read_data((MMC5983MARawData_t *)raw_data_storage);
      mmc5983ma_convert_and_calibrate(raw_data_storage, (MMC5983MABoardReading_t *)(&board_readings->magnetic_field_x_microteslas));
      break;
    case HIGH_G_ACCELEROMETER:
      high_g_read_data((ADXL371RawData_t *)raw_data_storage);
      adxl371_convert_and_calibrate(raw_data_storage, (ADXL371BoardReading_t *)(&board_readings->high_g_accel_x_gs));
  }

  board_readings->timestamp_seconds = process_clock_cycles(clock_cycles);
}
