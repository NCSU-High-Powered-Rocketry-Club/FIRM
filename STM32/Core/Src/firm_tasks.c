#include "firm_tasks.h"
#include "bmp581_packet.h"
#include "firm_fsm.h"
#include "icm45686_packet.h"
#include "led.h"
#include "mmc5983ma_packet.h"
#include "portmacro.h"
#include "stm32f4xx_hal_gpio.h"
#include "usb_print_debug.h"
#include <string.h>

// task handles
osThreadId_t system_manager_task_handle;
osThreadId_t firm_mode_indicator_task_handle;
osThreadId_t bmp581_task_handle;
osThreadId_t icm45686_task_handle;
osThreadId_t mmc5983ma_task_handle;
osThreadId_t usb_transmit_task_handle;
osThreadId_t uart_transmit_task_handle;
osThreadId_t usb_read_task_handle;
osThreadId_t command_handler_task_handle;
osThreadId_t filter_data_task_handle;

StreamBufferHandle_t usb_rx_stream;
QueueHandle_t usb_command_queue;
QueueHandle_t usb_response_queue;
QueueHandle_t system_request_queue;
QueueHandle_t mode_indicator_command_queue;
QueueHandle_t bmp581_command_queue;
QueueHandle_t icm45686_command_queue;
QueueHandle_t mmc5983ma_command_queue;
QueueHandle_t data_filter_command_queue;
QueueHandle_t usb_transmit_command_queue;
QueueHandle_t uart_transmit_command_queue;

// task attributes
const osThreadAttr_t systemManagerTask_attributes = {
    .name = "systemManagerTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
const osThreadAttr_t modeIndicatorTask_attributes = {
    .name = "modeIndicatorTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal
};
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
const osThreadAttr_t usbTask_attributes = {
    .name = "usbTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal1,
};
const osThreadAttr_t uartTask_attributes = {
    .name = "uartTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal1,
};
const osThreadAttr_t commandHandlerTask_attributes = {
    .name = "commandHandlerTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
const osThreadAttr_t filterDataTask_attributes = {
    .name = "filterDataTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

// mutexes
osMutexId_t sensorDataMutexHandle;
const osMutexAttr_t sensorDataMutex_attributes = {
  .name = "sensorDataMutex"
};

// instance of the unscented kalman filter
UKF ukf;

// instance of the data packet from the preprocessor to be reused
DataPacket_t data_packet = {0};

// instance of the serialized packet, will be reused
SerializedDataPacket_t serialized_packet = {0};

static UART_HandleTypeDef* firm_huart1;
static volatile bool uart_tx_done = true;

// A cancellation token that increments each time a cancel command is received
static volatile uint32_t command_cancel_token = 0;

// This just checks whether the command has been cancelled by comparing the
// snapshot of the token with the current cancellation token.
static bool is_command_cancelled(const CommandCancelCtx_t* cancel_context) {
  const CommandCancelCtx_t* c = cancel_context;
  return (c != NULL) && (c->cancel_seq != NULL) && (*(c->cancel_seq) != c->snapshot);
}

static void usb_on_parsed_command(const Command_t* command) {
  if (command == NULL) {
    return;
  }

  if (command->id == CMD_CANCEL_ID) {
    command_cancel_token++;
  }

  xQueueSend(usb_command_queue, command, 0);
}

void usb_receive_callback(uint8_t *buffer, uint32_t data_length) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Check if the stream buffer is initialized
  if (usb_rx_stream != NULL) {
    // Adds the received data to the stream buffer from an ISR context
    xStreamBufferSendFromISR(usb_rx_stream, buffer, data_length, &xHigherPriorityTaskWoken);
  }
  // If the usb_read_data task has a higher priority than the currently running task, request a context switch
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int initialize_firm(SPIHandles* spi_handles_ptr, I2CHandles* i2c_handles_ptr, DMAHandles* dma_handles_ptr, UARTHandles* uart_handles_ptr) {
  firm_huart1 = uart_handles_ptr->huart1;

  // We use DWT (Data Watchpoint and Trace unit) to get a high resolution free-running timer
  // for our data packet timestamps. This allows us to use the clock cycle count instead of a
  // standard timestamp in milliseconds or similar, while not having any performance penalty.
  // Enables the trace and debug block in the core so that DWT registers become
  // accessible. This is required before enabling the DWT cycle counter.
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  // Clear the DWT clock cycle counter to start counting from zero.
  DWT->CYCCNT = 0;

  // Enable the DWT cycle counter itself. Once active, it increments each CPU  
  // clock cycle so we can use clock cycles as data packet timestamps.
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Set the chip select pins to high, this means that they're not selected.
  // Note: We can't have these in the bmp581/imu/flash chip init functions, because those somehow
  // mess up with the initialization.
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // bmp581 cs pin
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // icm45686 cs pin
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); // flash chip cs pin
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // mmc5983ma CS pin

  // Indicate that initialization is in progress:
  led_set_status(FIRM_UNINITIALIZED);

  HAL_Delay(500); // purely for debug purposes, allows time to connect to USB serial terminal

  // disable the ISR so that the interrupts cannot be triggered before the scheduler initializes.
  // The ISR notifies the sensor tasks to collect data, but calling this before the scheduler is
  // initialized will suspend the program.
  HAL_NVIC_DisableIRQ(EXTI2_IRQn);
  HAL_NVIC_DisableIRQ(EXTI3_IRQn);
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

  if (icm45686_init(spi_handles_ptr->hspi2, GPIOB, GPIO_PIN_9)) {
    led_set_status(IMU_FAIL);
    return 1;
  }

  if (bmp581_init(spi_handles_ptr->hspi2, GPIOC, GPIO_PIN_2)) {
    led_set_status(BMP581_FAIL);
    return 1;
  }
  
  if (mmc5983ma_init(i2c_handles_ptr->hi2c1, 0x30)) {
    led_set_status(MMC5983MA_FAIL);
    return 1;
  }

  // set up settings module with flash chip
  if (settings_init(spi_handles_ptr->hspi1, GPIOC, GPIO_PIN_4)) {
    led_set_status(FLASH_CHIP_FAIL);
    return 1;
  }
  serializer_init_data_packet(&serialized_packet); // initializes the packet length and header bytes
  

  // // Wait for interrupts to fire
  // HAL_Delay(10);

  // // Check if interrupts fired
  // uint8_t interrupt_leds = 0b000;
  // // Blink LEDs to indicate any failed interrupts
  // for (int i = 0; i < 5; i++) {
  //     if (bmp581_has_new_data && icm45686_has_new_data && mmc5983ma_has_new_data) {
  //         break; // all interrupts fired successfully
  //     }
  //     if (!icm45686_has_new_data) {
  //         serialPrintStr("IMU didn't interrupt");
  //         interrupt_leds |= FAILED_INTERRUPT_IMU;
  //     }
  //     if (!bmp581_has_new_data) {
  //         serialPrintStr("BMP581 didn't interrupt");
  //         interrupt_leds |= FAILED_INTERRUPT_BMP;
  //     }
  //     if (!mmc5983ma_has_new_data) {
  //         serialPrintStr("MMC5983MA didn't interrupt");
  //         interrupt_leds |= FAILED_INTERRUPT_MAG;
  //     }
  //     led_set_status(interrupt_leds);
  //     HAL_Delay(500);
  //     led_set_status(FIRM_INITIALIZED);
  //     interrupt_leds = 0b000;
  //     HAL_Delay(500);
  // }
  return 0;
};

void firm_rtos_init(void) {
  // Initialize FreeRTOS objects
  // queue for system commands
  system_request_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(SystemRequest));
  // 512 bytes buffer, trigger level 1 (wake up on every byte if needed, or optimize later)
  usb_rx_stream = xStreamBufferCreate(USB_RX_STREAM_BUFFER_SIZE_BYTES, USB_RX_STREAM_TRIGGER_LEVEL_BYTES);
  // Queue for parsed commands
  usb_command_queue = xQueueCreate(USB_COMMAND_QUEUE_LENGTH, sizeof(Command_t));
  // Queue for response packets (66 bytes each): [A5 5A][len][pad][payload][crc]
  usb_response_queue = xQueueCreate(USB_RESPONSE_QUEUE_LENGTH, COMMAND_RESPONSE_PACKET_SIZE_BYTES);

  // queue for each task's commands
  bmp581_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  icm45686_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  mmc5983ma_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  data_filter_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  usb_transmit_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  uart_transmit_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  mode_indicator_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
}

void system_manager_task(void *argument) {
  SystemRequest sysreq;
  TaskCommand task_command_queue[MAX_TASK_COMMANDS];
  for (;;) {
    // wait to recieve a system request
    if (xQueueReceive(system_request_queue, &sysreq, portMAX_DELAY) == pdTRUE) {
      // pass system request to FSM, confirm that it's valid
      if (!fsm_process_request(sysreq, task_command_queue)) {
        // send command to the appropriate task
        for (int i = 0; i < MAX_TASK_COMMANDS; i++) {
          TaskCommand cmd = task_command_queue[i];
          if (cmd.target_task == TASK_NULL) {
            break;
          }

          switch (cmd.target_task) {
            case TASK_BMP581:
              xQueueSend(bmp581_command_queue, &cmd.command, 0);
              break;
            case TASK_ICM45686:
              xQueueSend(icm45686_command_queue, &cmd.command, 0);
              break;
            case TASK_MMC5983MA:
              xQueueSend(mmc5983ma_command_queue, &cmd.command, 0);
              break;
            case TASK_DATA_FILTER:
              xQueueSend(data_filter_command_queue, &cmd.command, 0);
              break;
            case TASK_USB_TRANSMIT:
              xQueueSend(usb_transmit_command_queue, &cmd.command, 0);
              break;
            case TASK_UART_TRANSMIT: 
              xQueueSend(uart_transmit_command_queue, &cmd.command, 0);
              break;
            case TASK_MODE_INDICATOR:
              xQueueSend(mode_indicator_command_queue, &cmd.command, 0);
            case TASK_NULL: {
              break;
            }
            default:
              break;
          }
        }
      }
    }
  }
}

void firm_mode_indicator_task(void *argument) {
  TaskCommandOption cmd_status;
  LED_Mode_Indicator_Status led_setting = FIRM_MODE_DEFAULT;
  led_set_status(led_setting);
  for (;;) {
    if (xQueueReceive(mode_indicator_command_queue, &cmd_status, pdMS_TO_TICKS(100)) == pdTRUE) {
      switch (cmd_status) {
        case TASKCMD_SETUP:
          led_setting = FIRM_MODE_BOOT;
          break;
        case TASKCMD_LIVE:
          led_setting = FIRM_MODE_LIVE;
          break;
        case TASKCMD_MOCK:
          led_setting = FIRM_MODE_MOCK;
          break;
        case TASKCMD_RESET: {
          led_setting = FIRM_MODE_DEFAULT;
          break;
        }
        default:
          led_setting = FIRM_MODE_DEFAULT;
      }
    }
    led_toggle_status(led_setting);
  }
}

void collect_bmp581_data_task(void *argument) {
  const TickType_t max_wait = MAX_WAIT_TIME(BMP581_POLL_RATE_HZ);
  uint32_t notif_count = 0;

  for (;;) {
    notif_count = ulTaskNotifyTake(pdFALSE, max_wait);

    if (notif_count > 0) {
      osMutexAcquire(sensorDataMutexHandle, osWaitForever);
      BMP581Packet_t *bmp581_packet = logger_malloc_packet(sizeof(BMP581Packet_t));
      if (!bmp581_read_data(bmp581_packet)) {
        logger_write_entry('B', sizeof(BMP581Packet_t));
        bmp581_convert_packet(bmp581_packet, &data_packet);
      }
      osMutexRelease(sensorDataMutexHandle);
    } else {
      // TODO: error handling
    }
  }
}

void collect_icm45686_data_task(void *argument) {
  const TickType_t max_wait = MAX_WAIT_TIME(ICM45686_POLL_RATE_HZ);
  uint32_t notif_count = 0;
  for (;;) {
    notif_count = ulTaskNotifyTake(pdFALSE, max_wait);

    if (notif_count > 0) {
      osMutexAcquire(sensorDataMutexHandle, osWaitForever);
      ICM45686Packet_t *icm45686_packet = logger_malloc_packet(sizeof(ICM45686Packet_t));
      if (!icm45686_read_data(icm45686_packet)) {
        logger_write_entry('I', sizeof(ICM45686Packet_t));
        icm45686_convert_packet(icm45686_packet, &data_packet);
      }
      osMutexRelease(sensorDataMutexHandle);
    } else {
      // TODO: error handling
    }
  }
}

void collect_mmc5983ma_data_task(void *argument) {
  const TickType_t max_wait = MAX_WAIT_TIME(ICM45686_POLL_RATE_HZ);
  uint32_t notif_count = 0;
  for (;;) {
    notif_count = ulTaskNotifyTake(pdFALSE, max_wait);

    if (notif_count > 0) {
      osMutexAcquire(sensorDataMutexHandle, osWaitForever);
      MMC5983MAPacket_t *mmc5983ma_packet = logger_malloc_packet(sizeof(MMC5983MAPacket_t));
      if (!mmc5983ma_read_data(mmc5983ma_packet)) {
        logger_write_entry('M', sizeof(MMC5983MAPacket_t));
        mmc5983ma_convert_packet(mmc5983ma_packet, &data_packet);
      }
      osMutexRelease(sensorDataMutexHandle);
    } else {
      // TODO: error handling
    }
  }
}

void filter_data_task(void *argument) {
  // time that FIRM should be running for (collecting sensor data) before starting the
  // kalman filter
  vTaskDelay(pdMS_TO_TICKS(KALMAN_FILTER_STARTUP_DELAY_TIME_MS));

  ukf_init(&ukf, data_packet.pressure, &data_packet.accel_x, &data_packet.magnetic_field_x);
  
  // set the last time to calculate the delta timestamp, minus some initial offset so that the
  // first iteration of the filter doesn't have an extremely small dt.
  float last_time = (float)data_packet.timestamp_sec - 0.005F;

  for (;;) {
    vTaskDelay(1000);
    float dt = (float)data_packet.timestamp_sec - last_time;
    ukf_predict(&ukf, dt);
    // float measurement[10] = {
    //   data_packet.pressure,
    //   data_packet.accel_x,
    //   data_packet.accel_y,
    //   data_packet.accel_z,
    //   data_packet.angular_rate_x,
    //   data_packet.angular_rate_y,
    //   data_packet.angular_rate_z,
    //   data_packet.magnetic_field_x,
    //   data_packet.magnetic_field_y,
    //   data_packet.magnetic_field_z,
    // };
    // ukf_update(&ukf, measurement);
    // memcpy(&data_packet.est_position_x, ukf.X, UKF_STATE_DIMENSION * 4);
    // last_time += dt;
  }
}

void command_handler_task(void *argument) {
  Command_t command;
  uint8_t payload_buffer[COMMAND_PAYLOAD_MAX_LEN_BYTES];
  uint8_t payload_length;
  uint8_t response_packet[COMMAND_RESPONSE_PACKET_SIZE_BYTES];

  CommandCancelCtx_t cancel_context = {
      .cancel_seq = &command_cancel_token,
      .snapshot = 0,
  };
  CommandContext_t command_context = {
      .is_cancelled = is_command_cancelled,
      .cancel_context = &cancel_context,
  };

  for (;;) {
    if (xQueueReceive(usb_command_queue, &command, portMAX_DELAY) == pdTRUE) {
      // One command at a time: run the command to completion before taking the next.
      cancel_context.snapshot = command_cancel_token;

      // Dispatch: command meanings live in commands.c; this task handles queuing/serialization.
      commands_handle_command(&command, &command_context, payload_buffer, &payload_length);
      
      // Serialize the response
      serialize_command_packet(payload_buffer, payload_length, response_packet);
      
      // Send to response queue
      xQueueSend(usb_response_queue, response_packet, 0);
    }
  }
}

void usb_transmit_data(void *argument) {
  
  const TickType_t transmit_freq = MAX_WAIT_TIME(TRANSMIT_FREQUENCY_HZ);
  TickType_t lastWakeTime = xTaskGetTickCount();
  uint8_t response_packet[COMMAND_RESPONSE_PACKET_SIZE_BYTES];
  
  for (;;) {
    // Check for response packets first
    if (xQueueReceive(usb_response_queue, response_packet, 0) == pdTRUE) {
      // Try to transmit until successful
      while (CDC_Transmit_FS(response_packet, COMMAND_RESPONSE_PACKET_SIZE_BYTES) == USBD_BUSY) {
        vTaskDelay(1); // Wait 1 tick before retrying
      }
    }

    // TODO: maybe consider having a queue for data packets too? Or not if it is too much slower.
    if (firmSettings.usb_transfer_enabled) {
      osMutexAcquire(sensorDataMutexHandle, osWaitForever);
      serialize_data_packet(&data_packet, &serialized_packet);
      osMutexRelease(sensorDataMutexHandle);
      
      usb_transmit_serialized_packet(&serialized_packet);
    }
    vTaskDelayUntil(&lastWakeTime, transmit_freq);
  }
}

void uart_transmit_data(void *argument) {
  const TickType_t transmit_freq = MAX_WAIT_TIME(TRANSMIT_FREQUENCY_HZ);
  TickType_t lastWakeTime = xTaskGetTickCount();
  
  for (;;) {
    if (firmSettings.uart_transfer_enabled) {
      osMutexAcquire(sensorDataMutexHandle, osWaitForever);
      serialize_data_packet(&data_packet, &serialized_packet);
      osMutexRelease(sensorDataMutexHandle);
      if (uart_tx_done) {
        uart_tx_done = false;
        HAL_UART_Transmit_DMA(firm_huart1, (uint8_t*)&serialized_packet, (uint16_t)sizeof(SerializedDataPacket_t));
      }
    }
    vTaskDelayUntil(&lastWakeTime, transmit_freq);
  }
}

void usb_read_data(void *argument) {
  uint8_t received_bytes[COMMAND_READ_CHUNK_SIZE_BYTES];
  CommandsStreamParser_t command_parser = { .len = 0 };

  for (;;) {
    size_t number_of_bytes_received = xStreamBufferReceive(usb_rx_stream, received_bytes, sizeof(received_bytes), portMAX_DELAY);
    if (number_of_bytes_received == 0) {
      continue;
    }

    // Adds the new bytes to the command parser and adds any parsed commands to the command queue
    commands_update_parser(&command_parser, received_bytes, number_of_bytes_received, usb_on_parsed_command);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == firm_huart1)
    uart_tx_done = true;
}
