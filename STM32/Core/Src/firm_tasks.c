#include "firm_tasks.h"
#include "bmp581_packet.h"
#include "firm_fsm.h"
#include "icm45686_packet.h"
#include "led.h"
#include "mmc5983ma_packet.h"
#include "mocking_handler.h"
#include "usb_print_debug.h"
#include "usbd_cdc_if.h"
#include <string.h>

#include "stm32f4xx_hal_cortex.h"

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
QueueHandle_t packetizer_command_queue;

// mock packets
// For mock mode, we receive raw bytes over USB that contain:
//   [timestamp bytes][sensor payload bytes]
// These buffers must be real storage (not pointers) and large enough for the
// timestamp + each sensor's payload.
#define SENSOR_TIMESTAMP_SIZE_BYTES (sizeof(((SensorPacket*)0)->timestamp))
static uint8_t mock_bmp581_payload[SENSOR_TIMESTAMP_SIZE_BYTES + sizeof(BMP581Packet_t)];
static uint8_t mock_icm45686_payload[SENSOR_TIMESTAMP_SIZE_BYTES + sizeof(ICM45686Packet_t)];
static uint8_t mock_mmc5983ma_payload[SENSOR_TIMESTAMP_SIZE_BYTES + sizeof(MMC5983MAPacket_t)];


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
    .stack_size = 5120 * 4,
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
const osMutexAttr_t sensorDataMutex_attributes = {
  .name = "sensorDataMutex"
};


// instance of the serialized verison of the data packet that will be reused and overriden as data
// is collected and processed
Packet data_packet;

static UART_HandleTypeDef* firm_huart1;
static volatile bool uart_tx_done = true;

static void firm_system_reset_cb(void *ctx) {
  (void)ctx;
  HAL_NVIC_SystemReset();
}

static bool mock_settings_write_cb(void *ctx, FIRMSettings_t *firm_settings, CalibrationSettings_t *calibration_settings) {
  (void)ctx;
  return settings_write_mock_settings(firm_settings, calibration_settings);
}

int initialize_firm(SPIHandles* spi_handles_ptr, I2CHandles* i2c_handles_ptr, DMAHandles* dma_handles_ptr, UARTHandles* uart_handles_ptr) {
  firm_huart1 = uart_handles_ptr->huart1;

  commands_register_system_reset(firm_system_reset_cb, NULL);

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
  return 0;
};

void firm_rtos_init(void) {
  // Initialize FreeRTOS objects
  // queue for system commands
  system_request_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(SystemRequest));
  
  // stream and queue for receiving usb bytes and sending packets
  usb_rx_stream = xStreamBufferCreate(USB_RX_STREAM_BUFFER_SIZE_BYTES, USB_RX_STREAM_TRIGGER_LEVEL_BYTES);
  transmit_queue = xQueueCreate(TRANSMIT_QUEUE_LENGTH, sizeof(Packet));

  // queue for each task's commands
  bmp581_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  icm45686_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  mmc5983ma_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  data_filter_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  mode_indicator_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(TaskCommandOption));
  packetizer_command_queue = xQueueCreate(SYSTEM_REQUEST_QUEUE_LENGTH, sizeof(SystemResponsePacket));
}

void system_manager_task(void *argument) {
  SystemRequest sysreq;
  TaskCommand task_command_queue[MAX_TASK_COMMANDS];

  for (;;) {
    // wait to recieve a system request
    if (xQueueReceive(system_request_queue, &sysreq, portMAX_DELAY) == pdTRUE) {
      // pass system request to FSM, confirm that it's valid
      FSMResponse response = fsm_process_request(sysreq, task_command_queue);
      SystemResponsePacket sys_response = {
        .identifier = sysreq,
        .success = (response == FSMRES_VALID)
      };
      
      if (!response) {
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
              break;
            case TASK_PACKETIZER:
              xQueueSend(packetizer_command_queue, &sys_response, 0);
              break;
            case TASK_NULL: {
              break;
            }
            default:
              break;
          }
        }
        
        // If cancel was successful, no file switching needed - continue logging to same file
      } else {
        // FSM returned invalid, send failure response
        xQueueSend(packetizer_command_queue, &sys_response, 0);
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
      led_set_status(FIRM_MODE_DEFAULT);
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
  TaskCommandOption cmd_status;

  for (;;) {
    xQueueReceive(bmp581_command_queue, &cmd_status, 0);

    notif_count = ulTaskNotifyTake(pdFALSE, max_wait);
    if (notif_count > 0) {
      osMutexAcquire(sensorDataMutexHandle, osWaitForever);
      void *bmp581_storage = logger_malloc_packet(sizeof(BMP581Packet_t) + 5);
      if (bmp581_storage == NULL) {
        osMutexRelease(sensorDataMutexHandle);
        continue;
      }
      SensorPacket *bmp581_packet = (SensorPacket *)((uint8_t *)bmp581_storage + 1);
      int err = 0;
      if (cmd_status == TASKCMD_MOCK) {
        memcpy(bmp581_packet, mock_bmp581_payload, SENSOR_TIMESTAMP_SIZE_BYTES + sizeof(BMP581Packet_t));
      } else {
        err = bmp581_read_data(&bmp581_packet->packet.bmp581_packet);
        uint32_t clock_cycle_count = DWT->CYCCNT;
        memcpy(bmp581_packet->timestamp, &clock_cycle_count, sizeof(bmp581_packet->timestamp));
      }
      if (!err) {
        logger_write_entry('B', sizeof(BMP581Packet_t));
        bmp581_convert_packet(bmp581_packet, (DataPacket *)&data_packet.data);
      }
      osMutexRelease(sensorDataMutexHandle);
    }
  }
}

void collect_icm45686_data_task(void *argument) {
  const TickType_t max_wait = MAX_WAIT_TIME(ICM45686_POLL_RATE_HZ);
  uint32_t notif_count = 0;
  TaskCommandOption cmd_status;

  for (;;) {
    xQueueReceive(icm45686_command_queue, &cmd_status, 0);

    notif_count = ulTaskNotifyTake(pdFALSE, max_wait);

    if (notif_count > 0) {
      osMutexAcquire(sensorDataMutexHandle, osWaitForever);
      void *icm45686_storage = logger_malloc_packet(sizeof(ICM45686Packet_t) + 5);
      if (icm45686_storage == NULL) {
        osMutexRelease(sensorDataMutexHandle);
        continue;
      }
      SensorPacket *icm45686_packet = (SensorPacket *)((uint8_t *)icm45686_storage + 1);
      int err = 0;
      if (cmd_status == TASKCMD_MOCK) {
        memcpy(icm45686_packet, mock_icm45686_payload, SENSOR_TIMESTAMP_SIZE_BYTES + sizeof(ICM45686Packet_t));
      } else {
        err = icm45686_read_data(&icm45686_packet->packet.icm45686_packet);
        uint32_t clock_cycle_count = DWT->CYCCNT;
        memcpy(icm45686_packet->timestamp, &clock_cycle_count, sizeof(icm45686_packet->timestamp));
      }
      if (!err) {
        logger_write_entry('I', sizeof(ICM45686Packet_t));
        icm45686_convert_packet(icm45686_packet, (DataPacket *)&data_packet.data);
      }
      osMutexRelease(sensorDataMutexHandle);
    }
  }
}

void collect_mmc5983ma_data_task(void *argument) {
  const TickType_t max_wait = MAX_WAIT_TIME(MMC5983MA_POLL_RATE_HZ);
  uint32_t notif_count = 0;
  TaskCommandOption cmd_status;

  for (;;) {
    xQueueReceive(mmc5983ma_command_queue, &cmd_status, 0);

    notif_count = ulTaskNotifyTake(pdFALSE, max_wait);

    if (notif_count > 0) {
      
      osMutexAcquire(sensorDataMutexHandle, osWaitForever);
      void *mmc5983ma_storage = logger_malloc_packet(sizeof(MMC5983MAPacket_t) + 5);
      if (mmc5983ma_storage == NULL) {
        osMutexRelease(sensorDataMutexHandle);
        continue;
      }
      SensorPacket *mmc5983ma_packet = (SensorPacket *)((uint8_t *)mmc5983ma_storage + 1);
      int err = 0;
      if (cmd_status == TASKCMD_MOCK) {
        memcpy(mmc5983ma_packet, mock_mmc5983ma_payload, SENSOR_TIMESTAMP_SIZE_BYTES + sizeof(MMC5983MAPacket_t));
      } else {
        err = mmc5983ma_read_data(&mmc5983ma_packet->packet.mmc5983ma_packet);
        uint32_t clock_cycle_count = DWT->CYCCNT;
        memcpy(mmc5983ma_packet->timestamp, &clock_cycle_count, sizeof(mmc5983ma_packet->timestamp));
      }
      if (!err) {
        logger_write_entry('M', sizeof(MMC5983MAPacket_t));
        mmc5983ma_convert_packet(mmc5983ma_packet, (DataPacket *)&data_packet.data);
      }
      osMutexRelease(sensorDataMutexHandle);
    }
  }
}

void filter_data_task(void *argument) {
  
  UKF ukf;
  ukf.measurement_function = ukf_measurement_function;
  ukf.state_transition_function = ukf_state_transition_function;
  vTaskDelay(pdMS_TO_TICKS(KALMAN_FILTER_STARTUP_DELAY_TIME_MS));
  DataPacket *packet = &data_packet.data.data_packet;
  TaskCommandOption cmd_status = TASKCMD_SETUP;
  float last_time; 

  for (;;) {
    xQueueReceive(data_filter_command_queue, &cmd_status, 0);
    if (cmd_status == TASKCMD_SETUP || cmd_status == TASKCMD_MOCK) {
      // time that FIRM should be running for (collecting sensor data) before starting the kalman filter
      vTaskDelay(pdMS_TO_TICKS(KALMAN_FILTER_STARTUP_DELAY_TIME_MS));
      // filter can initialize, and FIRM can go into live mode
      ukf_init(&ukf, packet->pressure_pascals, &packet->raw_acceleration_x_gs, &packet->magnetic_field_x_microteslas);
      // set the last time to calculate the delta timestamp, minus some initial offset so that the
      // first iteration of the filter doesn't have an extremely small dt.
      last_time = (float)packet->timestamp_seconds - 0.005F;
      if (cmd_status == TASKCMD_SETUP) {
        xQueueSend(system_request_queue, &(SystemRequest){SYSREQ_FINISH_SETUP}, portMAX_DELAY);
      }
    }
    if (cmd_status != TASKCMD_SETUP) {
      float dt = (float)packet->timestamp_seconds - last_time;
      int err = ukf_predict(&ukf, dt);
      ukf_set_measurement(&ukf, &packet->pressure_pascals);
      err = ukf_update(&ukf);
      if (err) {
        led_set_status(UKF_FAIL);
      }
      //osMutexAcquire(sensorDataMutexHandle, osWaitForever);
      memcpy(&packet->est_position_x_meters, ukf.X, UKF_STATE_DIMENSION * 4);
      //osMutexRelease(sensorDataMutexHandle);
      
      last_time += dt;
    }
  }
}

void packetizer_task(void *argument) {
  // initialize the persistent data packet with the length and identifier fields
  data_packet.header = MSGID_DATA_PACKET;
  data_packet.identifier = 0x0000;
  data_packet.packet_len = sizeof(DataPacket);
  // set the timer based on the set packet transmission frequency
  const TickType_t transmit_freq = MAX_WAIT_TIME(TRANSMIT_FREQUENCY_HZ);
  TickType_t last_wake_time = xTaskGetTickCount();
  SystemResponsePacket sys_response;

  for (;;) {
    if (xQueueReceive(packetizer_command_queue, &sys_response, 0) == pdTRUE) {
      Packet response;
      response.header = MSGID_RESPONSE_PACKET;
      response.identifier = sys_response.identifier;
      response.packet_len = sizeof(response.data.response_packet.success.b);
      response.data.response_packet.success.b = sys_response.success;
      xQueueSend(transmit_queue, &response, 0);
    }
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
      size_t serialized_packet_len = packet.packet_len + sizeof(packet.header) + sizeof(packet.identifier) + sizeof(packet.packet_len) + sizeof(packet.crc);
      packet.crc = crc16_ccitt((uint8_t *)&(packet), serialized_packet_len - 2);
      // move the location of the crc bytes directly after the payload
      memmove((uint8_t*)&packet + serialized_packet_len - 2, &packet.crc, sizeof(packet.crc));
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
  uint8_t meta_bytes[sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint32_t)];
  uint8_t received_bytes[COMMAND_READ_CHUNK_SIZE_BYTES];
  UsbMessageMeta meta;

  for (;;) {
    // simply an overflow check that can be removed later
    if (xStreamBufferBytesAvailable(usb_rx_stream) > 1500) {
      led_set_status(SD_CARD_FAIL);
    }

    xStreamBufferReceive(usb_rx_stream, meta_bytes, sizeof(meta_bytes), portMAX_DELAY);
    if (!usb_parse_message_meta(meta_bytes, sizeof(meta_bytes), &meta)) {
      continue;
    }

    xStreamBufferReceive(usb_rx_stream, received_bytes, meta.payload_length + 2U, portMAX_DELAY);

    UsbMessageType type = usb_interpret_usb_message(&meta, received_bytes, meta.payload_length + 2U);
    switch (type) {
      case USBMSG_MOCK_SENSOR: {
        size_t copy_len = (size_t)meta.payload_length;
        if (meta.identifier == (uint16_t)MOCKID_BMP581) {
          memcpy(mock_bmp581_payload, received_bytes, copy_len);
          xTaskNotifyGive(bmp581_task_handle);
        } else if (meta.identifier == (uint16_t)MOCKID_ICM45686) {
          memcpy(mock_icm45686_payload, received_bytes, copy_len);
          xTaskNotifyGive(icm45686_task_handle);
        } else if (meta.identifier == (uint16_t)MOCKID_MMC5983MA) {
          memcpy(mock_mmc5983ma_payload, received_bytes, copy_len);
          xTaskNotifyGive(mmc5983ma_task_handle);
        }
        break;
      }
      case USBMSG_MOCK_SETTINGS: {
        FIRMSettings_t mock_firm_settings;
        CalibrationSettings_t mock_calibration_settings;
        HeaderFields header_fields;
        if (process_mock_settings_packet(received_bytes, meta.payload_length, &mock_firm_settings, &mock_calibration_settings, &header_fields, mock_settings_write_cb, NULL)) {
          (void)logger_append_mock_header(&mock_firm_settings, &mock_calibration_settings, &header_fields);
        }
        break;
      }
      case USBMSG_COMMAND: {
        Packet response;
        response.packet_len = commands_execute_to_response(meta.identifier, received_bytes, meta.payload_length, &response.header, &response.identifier, (ResponsePacket *)&response.data);
        xQueueSend(transmit_queue, &response, portMAX_DELAY);
        break;
      }
      case USBMSG_SYSTEM_REQUEST:
        xQueueSend(system_request_queue, (SystemRequest *)&meta.identifier, portMAX_DELAY);
        break;
      case USBMSG_INVALID:
      default:
        break;
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == firm_huart1)
    uart_tx_done = true;
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
