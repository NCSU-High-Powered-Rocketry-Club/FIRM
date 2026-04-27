#include "task_runtime.h"

#include "cmsis_os2.h"
#include "filter_data_task.h"
#include "mode_indicator_task.h"
#include "packetizer_task.h"
#include "sensor_task.h"
#include "transmit_task.h"
#include "usb_read_data_task.h"

#include "commands.h"
#include "data_processing/mocking_ring_buffer.h"
#include "led.h"
#include "logger.h"
#include "mocking_handler.h"
#include "semphr.h"
#include <string.h>

// task handles
osThreadId_t firm_mode_indicator_task_handle;
osThreadId_t filter_data_task_handle;
osThreadId_t packetizer_task_handle;
osThreadId_t transmit_task_handle;
osThreadId_t usb_read_task_handle;

// usb and transmit queues and streams
StreamBufferHandle_t usb_rx_stream;
QueueHandle_t transmit_queue;

// event for when all sensors have collected data
EventGroupHandle_t sensors_collected;

// task attributes
const osThreadAttr_t systemManagerTask_attributes = {
    .name = "systemManagerTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
const osThreadAttr_t modeIndicatorTask_attributes = {
    .name = "modeIndicatorTask", .stack_size = 128 * 4, .priority = (osPriority_t)osPriorityNormal};
const osThreadAttr_t sensorTask_attributes = {
    .name = "sensorTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh7,
};
const osThreadAttr_t filterDataTask_attributes = {
    .name = "filterDataTask",
    .stack_size = 4096 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
const osThreadAttr_t packetizerTask_attributes = {
    .name = "packetizerTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
const osThreadAttr_t transmitTask_attributes = {
    .name = "transmitTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal1,
};
const osThreadAttr_t usbReadTask_attributes = {
    .name = "usbReadTask",
    .stack_size = 2048 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

// mutexes
osMutexId_t sensorDataMutexHandle;
const osMutexAttr_t sensorDataMutex_attributes = {.name = "sensorDataMutex"};

static bool mock_count_try_take(void *context) {
  return xSemaphoreTake((SemaphoreHandle_t)context, 0U) == pdTRUE;
}

static bool mock_count_give(void *context) {
  return xSemaphoreGive((SemaphoreHandle_t)context) == pdTRUE;
}

static size_t mock_count_get_count(void *context) {
  return (size_t)uxSemaphoreGetCount((SemaphoreHandle_t)context);
}

static void mock_count_reset(void *context) {
  while (xSemaphoreTake((SemaphoreHandle_t)context, 0U) == pdTRUE) {
  }
}


void firm_rtos_init(void) {
  usb_rx_stream =
      xStreamBufferCreate(USB_RX_STREAM_BUFFER_SIZE_BYTES, USB_RX_STREAM_TRIGGER_LEVEL_BYTES);
  transmit_queue = xQueueCreate(TRANSMIT_QUEUE_LENGTH, sizeof(TransmitFrame_t));

  SemaphoreHandle_t mock_count_sem = xSemaphoreCreateCounting(MOCK_QUEUE_LENGTH, 0U);
  if (mock_count_sem != NULL) {
    MockRingCountSemaphore_t mock_counting_semaphore = {
        .context = mock_count_sem,
        .try_take = mock_count_try_take,
        .give = mock_count_give,
        .get_count = mock_count_get_count,
        .reset = mock_count_reset,
    };

    // get the curent clock frequency
    uint32_t hclk_hz = HAL_RCC_GetHCLKFreq();
    uint32_t hclk_mhz = (hclk_hz == 0U) ? 168U : (hclk_hz / 1000000U);
    mocking_handler_init(&mock_counting_semaphore, hclk_mhz);
  }
}
