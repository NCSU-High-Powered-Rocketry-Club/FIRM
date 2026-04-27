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

extern osThreadId_t bmp581_task_handle;
extern osThreadId_t icm45686_task_handle;
extern osThreadId_t mmc5983ma_task_handle;
extern osThreadId_t adxl371_task_handle;

extern const osThreadAttr_t bmp581Task_attributes;
extern const osThreadAttr_t icm45686Task_attributes;
extern const osThreadAttr_t mmc5983maTask_attributes;
extern const osThreadAttr_t adxl371Task_attributes;

extern QueueHandle_t bmp581_command_queue;
extern QueueHandle_t icm45686_command_queue;
extern QueueHandle_t mmc5983ma_command_queue;
extern QueueHandle_t adxl371_command_queue;

void collect_bmp581_data_task(void *argument);
void collect_icm45686_data_task(void *argument);
void collect_mmc5983ma_data_task(void *argument);
void collect_adxl371_data_task(void *argument);