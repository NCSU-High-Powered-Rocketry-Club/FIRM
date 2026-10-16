#pragma once
#include "clock_cycle_count.h"
#include "logger.h"
#include "packets.h"
#include "shared_data/identifiers.h"

#include "adxl371.h"
#include "bmp581.h"
#include "icm45686.h"
#include "mmc5983ma.h"

/**
 * @brief collects, logs, and processes data from the given sensor
 *
 * @param sensor the sensor type to collect data from
 * @param board_readings the instance of board-wide global data to output the new readings into
 */
void sensor_collect_data(Identifiers_t sensor, DataPacket_t *board_readings);

/**
 * @brief Tells the sensor manager how to get the timestamp for a sensor reading.
 *
 * @param time_fn function pointer that takes no args and returns an unsigned 32 bit timestamp.
 */
void set_time_fn(uint32_t (*time_fn)(void));
