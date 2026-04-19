#pragma once
#include "log_info.h"
#include "logger_storage.h"
#include "sensors.h"
#include "system_settings.h"

#include <stdio.h>

/**
 * @brief Creates a new log file on the storage system.
 * @note If a file is currently open, it will be closed.
 *
 * @retval 0 on success, 1 on error.
 */
int create_log(void);

/**
 * @brief Writes header information to the log file, containing on the system settings.
 *
 * @param header_info the SystemSettings that will be included in the log file header
 * @retval 0 on success, 1 on error.
 */
int logger_write_header(SystemSettings_t header_info);

/**
 * @brief Allocates storage in the log file containing the number of bytes required for the given
 *        sensor's raw data.
 * @note Must ensure to call logger_set_sensor_info to tell the logger module the size of each
 *       sensor struct, before calling this function. This function assumes the the timestamp is 4
 *       bytes. Note that this function will automatically increment the internal cursor pointer
 *       in the file.
 *
 * @param sensor_id The ID of the sensor to allocate space for. This is also the identifier byte
 *                  that will be written at the start of the entry for this data in the log file.
 * @param timestamp The 32-bit unsigned timestamp for when the sensor data was collected.
 * @retval void pointer to the allocated memory.
 */
void *logger_malloc_raw_storage(Sensors_t sensor_id, uint32_t timestamp);

/**
 * @brief Tells the logger module how big each sensor raw data struct is that will be logged.
 * @note This info must be set before logger_malloc_raw_storage is called.
 *
 * @param barometer_size the size in bytes of the barometer raw data
 * @param imu_size the size in bytes of the imu raw data
 * @param magnetometer_size the size in bytes of the magnetometer raw data
 * @param high_g_size the size in bytes of the high-g accelerometer raw data
 */
void logger_set_sensor_info(size_t barometer_size, size_t imu_size, size_t magnetometer_size,
                            size_t high_g_size);