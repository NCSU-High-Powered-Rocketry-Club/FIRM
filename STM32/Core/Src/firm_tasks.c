#include "firm_tasks.h"
#include "bmp581_packet.h"
#include "firm_fsm.h"
#include "icm45686_packet.h"
#include "led.h"
#include "mmc5983ma_packet.h"
#include "usb_print_debug.h"
#include "usbd_cdc_if.h"
#include <string.h>

// task handles
osThreadId_t system_manager_task_handle;
osThreadId_t firm_mode_indicator_task_handle;
osThreadId_t bmp581_task_handle;
osThreadId_t icm45686_task_handle;
osThreadId_t mmc5983ma_task_handle;
osThreadId_t filter_data_task_handle;
osThreadId_t packetizer_task_handle;
osThreadId_t transmit_task_handle;
osThreadId_t usb_read_task_handle;


StreamBufferHandle_t usb_rx_stream;
QueueHandle_t transmit_queue;

QueueHandle_t system_request_queue;
QueueHandle_t mode_indicator_command_queue;
QueueHandle_t bmp581_command_queue;
QueueHandle_t icm45686_command_queue;
QueueHandle_t mmc5983ma_command_queue;
QueueHandle_t data_filter_command_queue;


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
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal1,
};
const osThreadAttr_t usbReadTask_attributes = {
    .name = "usbReadTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};


// mutexes
osMutexId_t sensorDataMutexHandle;
const osMutexAttr_t sensorDataMutex_attributes = {
  .name = "sensorDataMutex"
};

// instance of the serialized verison of the data packet that will be reused and overriden as data
// is collected and processed
Packet data_packet;

static UART_HandleTypeDef* firm_huart1;
static volatile bool uart_tx_done = true;

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
  transmit_queue = xQueueCreate(TRANSMIT_QUEUE_LENGTH, sizeof(Packet));

  // queue for each task's commands
  bmp581_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  icm45686_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  mmc5983ma_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  data_filter_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
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
        bmp581_convert_packet(bmp581_packet, (DataPacket *)&data_packet.data);
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
        icm45686_convert_packet(icm45686_packet, (DataPacket *)&data_packet.data);
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
        mmc5983ma_convert_packet(mmc5983ma_packet, (DataPacket *)&data_packet.data);
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
  UKF ukf;
  ukf.measurement_function = ukf_measurement_function;
  ukf.state_transition_function = ukf_state_transition_function;
  vTaskDelay(pdMS_TO_TICKS(KALMAN_FILTER_STARTUP_DELAY_TIME_MS));
  DataPacket *packet = (DataPacket *)&data_packet.data;
  
  // filter can initialize, and FIRM can go into live mode
  ukf_init(&ukf, packet->pressure_pascals, &packet->est_acceleration_x_gs, &packet->magnetic_field_x_microteslas);
  xQueueSend(system_request_queue, &(SystemRequest){SYSREQ_FINISH_SETUP}, 0);
  
  // set the last time to calculate the delta timestamp, minus some initial offset so that the
  // first iteration of the filter doesn't have an extremely small dt.
  float last_time = (float)packet->timestamp_seconds - 0.005F;

  for (;;) {
    vTaskDelay(1000);
    float dt = (float)packet->timestamp_seconds - last_time;
    ukf_predict(&ukf, dt);
    float measurement[10] = {
      packet->pressure_pascals,
      packet->raw_acceleration_x_gs,
      packet->raw_acceleration_y_gs,
      packet->raw_acceleration_z_gs,
      packet->raw_angular_rate_x_deg_per_s,
      packet->raw_angular_rate_y_deg_per_s,
      packet->raw_angular_rate_z_deg_per_s,
      packet->magnetic_field_x_microteslas,
      packet->magnetic_field_y_microteslas,
      packet->magnetic_field_z_microteslas,
    };
    ukf_update(&ukf, measurement);
    memcpy(&packet->est_position_x_meters, ukf.X, UKF_STATE_DIMENSION * 4);
    last_time += dt;
  }
}

void packetizer_task(void *argument) {
  // initialize the persistent data packet with the length and identifier fields
  data_packet.identifier = 0xA55AA55A;
  data_packet.packet_len = sizeof(DataPacket);

  // set the timer based on the set packet transmission frequency
  const TickType_t transmit_freq = MAX_WAIT_TIME(TRANSMIT_FREQUENCY_HZ);
  TickType_t last_wake_time = xTaskGetTickCount();
  for (;;) {
    // send to queue at the specified frequency
    xQueueSend(transmit_queue, &data_packet, 0);
    vTaskDelayUntil(&last_wake_time, transmit_freq);
  }
}

void transmit_data(void *argument) {
  Packet packet;
  for (;;) {
    if (xQueueReceive(transmit_queue, &packet, portMAX_DELAY) == pdTRUE) {
      // calculate and attach the crc16 to the end of the packet
      packet.crc = crc16_ccitt((uint8_t *)&(packet.data), packet.packet_len);
      uint16_t serialized_packet_len = packet.packet_len + sizeof(packet.identifier) + sizeof(packet.packet_len) + sizeof(packet.crc);
      // move the location of the crc bytes directly after the payload
      memmove(&packet + serialized_packet_len - 2, &packet.crc, sizeof(packet.crc));
      CDC_Transmit_FS((uint8_t*)&packet, serialized_packet_len);
      // optionally transmit over uart if the setting is enabled
      if (firmSettings.uart_transfer_enabled && uart_tx_done) {
        uart_tx_done = false;
        HAL_UART_Transmit_DMA(firm_huart1, (uint8_t*)&packet, serialized_packet_len);
      }
    }
  }
}

void usb_read_data(void *argument) {
  uint8_t received_bytes[COMMAND_READ_CHUNK_SIZE_BYTES];
  uint8_t command_buffer[CMD_PACKET_SIZE];
  CommandsStreamParser_t command_parser = { .len = 0 };

  for (;;) {
    // Receive incoming USB data
    size_t number_of_bytes_received = xStreamBufferReceive(usb_rx_stream, received_bytes, sizeof(received_bytes), portMAX_DELAY);
    if (number_of_bytes_received == 0) {
      continue;
    }

    // Feed bytes into parser
    commands_update_parser(&command_parser, received_bytes, number_of_bytes_received);

    // Process all complete commands in the parser
    while (commands_extract_next_packet(&command_parser, command_buffer)) {
      // Create response packet with ResponsePacket payload
      Packet response_packet;
      response_packet.identifier = 0xA55AA55A;
      
      // Process command and fill response payload
      if (commands_process_and_respond(command_buffer, (ResponsePacket*)&response_packet.data)) {
        // Calculate packet length based on response_id to determine actual payload size
        // For now, use sizeof(ResponsePacket) as placeholder - adjust based on actual response type if needed
        response_packet.packet_len = sizeof(ResponsePacket);
        
        // Queue for transmission (transmit_data will calculate CRC)
        xQueueSend(transmit_queue, &response_packet, 0);
      }
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == firm_huart1)
    uart_tx_done = true;
}
