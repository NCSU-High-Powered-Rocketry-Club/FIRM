#pragma once

#include "task_runtime.h"

#define BMP581_POLL_RATE_HZ 255
#define ICM45686_POLL_RATE_HZ 400
#define MMC5983MA_POLL_RATE_HZ 225
#define ADXL371_POLL_RATE_HZ 160

// bitmask for the sensor tasks (only ones used in kalman filter)
// TODO: include ADXL in kalman filter
#define BMP581_TASK_BIT (1 << 0)
#define ICM45686_TASK_BIT (1 << 1)
#define MMC5983MA_TASK_BIT (1 << 2)

extern osThreadId_t sensor_task_handle;
extern const osThreadAttr_t sensorTask_attributes;

void sensor_task(void *argument);