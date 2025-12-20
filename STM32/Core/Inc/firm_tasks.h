#ifndef FIRM_TASKS_H
#define FIRM_TASKS_H

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#define FIRM_TASK_DEFAULT_PRIORITY 100

#define BMP581_POLL_RATE_HZ 500
#define ICM45686_POLL_RATE_HZ 800
#define MMC5983MA_POLL_RATE_HZ 200

#define MAX_WAIT_TIME(hz) (TickType_t)(pdMS_TO_TICKS(1000/(hz))+1)

static QueueHandle_t dataQueue;

static TaskHandle_t hBMP581DataTask;
static TaskHandle_t hICM45686DataTask;
static TaskHandle_t hMMC5983MADataTask;

void TaskCollectBMP581Data(void *argument);
void TaskCollectICM45686Data(void *argument);
void TaskCollectMMC5983MAData(void *argument);

void TaskWriteSerialData(void *argument);

#endif //FIRM_TASKS_H