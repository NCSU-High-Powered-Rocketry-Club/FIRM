#include "task_runtime.h"

#include "filter_data_task.h"
#include "mock_packet_handler_task.h"
#include "mode_indicator_task.h"
#include "packetizer_task.h"
#include "sensor_task.h"
#include "system_manager_task.h"
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
osThreadId_t system_manager_task_handle;
osThreadId_t firm_mode_indicator_task_handle;
osThreadId_t bmp581_task_handle;
osThreadId_t icm45686_task_handle;
osThreadId_t mmc5983ma_task_handle;
osThreadId_t adxl371_task_handle;
osThreadId_t filter_data_task_handle;
osThreadId_t packetizer_task_handle;
osThreadId_t transmit_task_handle;
osThreadId_t usb_read_task_handle;
osThreadId_t mock_packet_handler_handle;

// usb and transmit queues and streams
StreamBufferHandle_t usb_rx_stream;
QueueHandle_t transmit_queue;

// command/request queues
QueueHandle_t system_request_queue;
QueueHandle_t mode_indicator_command_queue;
QueueHandle_t bmp581_command_queue;
QueueHandle_t icm45686_command_queue;
QueueHandle_t mmc5983ma_command_queue;
QueueHandle_t adxl371_command_queue;
QueueHandle_t data_filter_command_queue;
QueueHandle_t mock_packet_handler_command_queue;

// event for when all sensors have collected data
EventGroupHandle_t sensors_collected;

// shared data packet for task pipeline
DataPacket_t latest_data_packet;

// task attributes
const osThreadAttr_t systemManagerTask_attributes = {
    .name = "systemManagerTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
const osThreadAttr_t modeIndicatorTask_attributes = {
    .name = "modeIndicatorTask", .stack_size = 128 * 4, .priority = (osPriority_t)osPriorityNormal};
const osThreadAttr_t bmp581Task_attributes = {
    .name = "bmp581Task",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
const osThreadAttr_t icm45686Task_attributes = {
    .name = "icm45686Task",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
const osThreadAttr_t mmc5983maTask_attributes = {
    .name = "mmc5983maTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
const osThreadAttr_t adxl371Task_attributes = {
    .name = "adxl371Task",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh,
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
const osThreadAttr_t mockPacketTask_attributes = {
    .name = "mockPacketTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

// mutexes
osMutexId_t sensorDataMutexHandle;
const osMutexAttr_t sensorDataMutex_attributes = {.name = "sensorDataMutex"};

static void firm_system_reset_cb(void *ctx) {
  (void)ctx;
  HAL_NVIC_SystemReset();
}

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

int initialize_firm(SPIHandles *spi_handles_ptr, I2CHandles *i2c_handles_ptr,
                    DMAHandles *dma_handles_ptr, UARTHandles *uart_handles_ptr) {
  (void)i2c_handles_ptr;
  (void)dma_handles_ptr;
  (void)uart_handles_ptr;

  commands_register_system_reset(firm_system_reset_cb, NULL);

  // We use DWT (Data Watchpoint and Trace unit) to get a high resolution free-running timer.
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Ensure SPI chip-select lines default to not selected.
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // BMP581 CS pin
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // ICM45686 CS pin
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); // flash chip CS pin
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // MMC5983MA CS pin
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); // ADXL371 CS pin

  led_set_status(FIRM_UNINITIALIZED);

  HAL_Delay(100);

  // The scheduler is not running yet; prevent EXTI callbacks from notifying task handles.
  HAL_NVIC_DisableIRQ(EXTI0_IRQn);
  HAL_NVIC_DisableIRQ(EXTI1_IRQn);
  HAL_NVIC_DisableIRQ(EXTI2_IRQn);
  HAL_NVIC_DisableIRQ(EXTI3_IRQn);

  set_spi_icm(spi_handles_ptr->hspi2, GPIOB, GPIO_PIN_9);
  set_spi_mmc(spi_handles_ptr->hspi2, GPIOC, GPIO_PIN_7);
  set_spi_bmp(spi_handles_ptr->hspi2, GPIOC, GPIO_PIN_2);
  set_spi_adxl(spi_handles_ptr->hspi2, GPIOA, GPIO_PIN_8);

  if (icm45686_init()) {
    led_set_status(IMU_FAIL);
    Error_Handler();
    return 1;
  }

  if (mmc5983ma_init()) {
    led_set_status(MMC5983MA_FAIL);
    Error_Handler();
    return 1;
  }

  if (bmp581_init()) {
    led_set_status(BMP581_FAIL);
    Error_Handler();
    return 1;
  }

  if (adxl371_init()) {
    led_set_status(HIGH_G_FAIL);
    Error_Handler();
    return 1;
  }

  logger_set_sensor_info(sizeof(BMP581RawData_t), sizeof(ICM45686RawData_t),
                         sizeof(MMC5983MARawData_t), sizeof(ADXL371RawData_t));
  memset(&latest_data_packet, 0, sizeof(latest_data_packet));

  return 0;
}

void firm_rtos_init(void) {
  system_request_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(SystemRequest));

  usb_rx_stream =
      xStreamBufferCreate(USB_RX_STREAM_BUFFER_SIZE_BYTES, USB_RX_STREAM_TRIGGER_LEVEL_BYTES);
  transmit_queue = xQueueCreate(TRANSMIT_QUEUE_LENGTH, sizeof(TransmitFrame_t));

  bmp581_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  icm45686_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  mmc5983ma_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  adxl371_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  data_filter_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  mode_indicator_command_queue =
      xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  mock_packet_handler_command_queue =
      xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));

  sensors_collected = xEventGroupCreate();

  SemaphoreHandle_t mock_count_sem = xSemaphoreCreateCounting(MOCK_QUEUE_LENGTH, 0U);
  if (mock_count_sem != NULL) {
    MockRingCountSemaphore_t mock_counting_semaphore = {
        .context = mock_count_sem,
        .try_take = mock_count_try_take,
        .give = mock_count_give,
        .get_count = mock_count_get_count,
        .reset = mock_count_reset,
    };
    uint32_t hclk_hz = HAL_RCC_GetHCLKFreq();
    uint32_t hclk_mhz = (hclk_hz == 0U) ? 168U : (hclk_hz / 1000000U);
    mocking_handler_init(&mock_counting_semaphore, hclk_mhz);
  }
}
