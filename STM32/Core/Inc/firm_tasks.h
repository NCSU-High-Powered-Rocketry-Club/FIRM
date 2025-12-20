#ifndef FIRM_TASKS_H
#define FIRM_TASKS_H

#include "cmsis_os.h"

#define FIRM_TASK_DEFAULT_PRIORITY 100

#define BMP581_POLL_RATE_HZ 500
#define ICM45686_POLL_RATE_HZ 800
#define MMC5983MA_POLL_RATE_HZ 200

#define MAX_WAIT_TIME(hz) (TickType_t)(pdMS_TO_TICKS(1000/(hz))+1)

extern osThreadId_t bmp581_task_handle;
extern const osThreadAttr_t bmp581Task_attributes;
extern osThreadId_t icm45686_task_handle;
extern const osThreadAttr_t icm45686Task_attributes;
extern osThreadId_t mmc5983ma_task_handle;
extern const osThreadAttr_t mmc5983maTask_attributes;
extern osMutexId_t sensorDataMutexHandle;
extern const osMutexAttr_t sensorDataMutex_attributes;

void collect_bmp581_data_task(void *argument);
void collect_icm45686_data_task(void *argument);
void collect_mmc5983ma_data_task(void *argument);

void TaskWriteSerialData(void *argument);

#endif //FIRM_TASKS_H