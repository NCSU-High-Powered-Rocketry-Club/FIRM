#pragma once
#include "sensors.h"
#include "logger.h"
#include "clock_cycle_count.h"

#include "bmp581.h"
#include "icm45686.h"
#include "mmc5983ma.h"
#include "adxl371.h"

/**
 * @brief collects, logs, and processes data from the given sensor
 * 
 * @param sensor the sensor type to collect data from
 * @param board_readings the instance of board-wide global data to output the new readings into
 */
void sensor_collect_data(Sensors_t sensor, DataPacket_t *board_readings);

/**
 * @brief Tells the sensor manager how to get the timestamp for a sensor reading
 * @note Set by the mock manager
 * 
 * @param time_fn function pointer that takes no args and returns an unsigned 32 bit timestamp.
 */
void set_time_fn(uint32_t(*time_fn)(void));

/**
 * @brief Injects barometer raw read callback.
 */
void set_barometer_read_fn(int (*read_fn)(BMP581RawData_t *));

/**
 * @brief Injects IMU raw read callback.
 */
void set_imu_read_fn(int (*read_fn)(ICM45686RawData_t *));

/**
 * @brief Injects magnetometer raw read callback.
 */
void set_magnetometer_read_fn(int (*read_fn)(MMC5983MARawData_t *));

/**
 * @brief Injects high-g accelerometer raw read callback.
 */
void set_high_g_read_fn(int (*read_fn)(ADXL371RawData_t *));