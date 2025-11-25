#ifndef FIRM_TASKS_H
#define FIRM_TASKS_H

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#define FIRM_TASK_DEFAULT_PRIORITY 100

#define BMP_POLL_RATE_HZ 500
#define IMU_POLL_RATE_HZ 800
#define MAG_POLL_RATE_HZ 200

#define MAX_WAIT_TIME(hz) (TickType_t)(pdMS_TO_TICKS(1000/hz)+1)

static QueueHandle_t dataQueue;

static TaskHandle_t hBarDataTask;
static TaskHandle_t hImuDataTask;
static TaskHandle_t hMagDataTask;

void TaskCollectBarData(void *argument);
void TaskCollectIMUData(void *argument);
void TaskCollectMagData(void *argument);

void TaskWriteSerialData(void *argument);

#endif //FIRM_TASKS_H