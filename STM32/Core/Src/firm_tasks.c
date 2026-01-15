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
QueueHandle_t packetizer_command_queue;

// mock queues
QueueHandle_t mock_bmp581_queue;
QueueHandle_t mock_icm45686_queue;
QueueHandle_t mock_mmc5983ma_queue;


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
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal1,
};
const osThreadAttr_t usbReadTask_attributes = {
    .name = "usbReadTask",
    .stack_size = 4096 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

// mutexes
osMutexId_t sensorDataMutexHandle;
const osMutexAttr_t sensorDataMutex_attributes = {
  .name = "sensorDataMutex"
};

static int mock_fetch_sensor_queue(SensorPacket *packet, QueueHandle_t queue) {
  xQueueReceive(queue, packet, portMAX_DELAY);
  return 0;
}


// instance of the serialized verison of the data packet that will be reused and overriden as data
// is collected and processed
Packet data_packet;

static UART_HandleTypeDef* firm_huart1;
static volatile bool uart_tx_done = true;

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

  // queue for mock packets
  mock_bmp581_queue = xQueueCreate(MOCK_QUEUE_LENGTH, sizeof(SensorPacket));
  mock_icm45686_queue = xQueueCreate(MOCK_QUEUE_LENGTH, sizeof(SensorPacket));
  mock_mmc5983ma_queue = xQueueCreate(MOCK_QUEUE_LENGTH, sizeof(SensorPacket));
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
      SensorPacket *bmp581_packet = logger_malloc_packet(sizeof(BMP581Packet_t));
      int err;
      if (cmd_status == TASKCMD_MOCK) {
        err = mock_fetch_sensor_queue(bmp581_packet, mock_bmp581_queue);
      } else {
        err = bmp581_read_data(&bmp581_packet->packet.bmp581_packet);
        // TODO: change timestamp format to little endian
        uint32_t clock_cycle_count = DWT->CYCCNT;
        bmp581_packet->timestamp[0] = (clock_cycle_count >> 24) & 0xFF;
        bmp581_packet->timestamp[1] = (clock_cycle_count >> 16) & 0xFF;
        bmp581_packet->timestamp[2] = (clock_cycle_count >> 8)  & 0xFF;
        bmp581_packet->timestamp[3] =  clock_cycle_count & 0xFF;
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
      SensorPacket *icm45686_packet = logger_malloc_packet(sizeof(ICM45686Packet_t));
      int err;
      if (cmd_status == TASKCMD_MOCK) {
        err = mock_fetch_sensor_queue(icm45686_packet, mock_icm45686_queue);
      } else {
        err = icm45686_read_data(&icm45686_packet->packet.icm45686_packet);
        // TODO: change timestamp format to little endian
        uint32_t clock_cycle_count = DWT->CYCCNT;
        icm45686_packet->timestamp[0] = (clock_cycle_count >> 24) & 0xFF;
        icm45686_packet->timestamp[1] = (clock_cycle_count >> 16) & 0xFF;
        icm45686_packet->timestamp[2] = (clock_cycle_count >> 8)  & 0xFF;
        icm45686_packet->timestamp[3] =  clock_cycle_count & 0xFF;
      }
      if (!err) {
        logger_write_entry('B', sizeof(ICM45686Packet_t));
        icm45686_convert_packet(icm45686_packet, (DataPacket *)&data_packet.data);
      }
      osMutexRelease(sensorDataMutexHandle);
    }
  }
}

void collect_mmc5983ma_data_task(void *argument) {
  const TickType_t max_wait = MAX_WAIT_TIME(ICM45686_POLL_RATE_HZ);
  uint32_t notif_count = 0;
  TaskCommandOption cmd_status;

  for (;;) {
    xQueueReceive(mmc5983ma_command_queue, &cmd_status, 0);

    notif_count = ulTaskNotifyTake(pdFALSE, max_wait);

    if (notif_count > 0) {
      osMutexAcquire(sensorDataMutexHandle, osWaitForever);
      SensorPacket *mmc5983ma_packet = logger_malloc_packet(sizeof(MMC5983MAPacket_t));
      int err;
      if (cmd_status == TASKCMD_MOCK) {
        err = mock_fetch_sensor_queue(mmc5983ma_packet, mock_mmc5983ma_queue);
      } else {
        err = mmc5983ma_read_data(&mmc5983ma_packet->packet.mmc5983ma_packet);
        // TODO: change timestamp format to little endian
        uint32_t clock_cycle_count = DWT->CYCCNT;
        mmc5983ma_packet->timestamp[0] = (clock_cycle_count >> 24) & 0xFF;
        mmc5983ma_packet->timestamp[1] = (clock_cycle_count >> 16) & 0xFF;
        mmc5983ma_packet->timestamp[2] = (clock_cycle_count >> 8)  & 0xFF;
        mmc5983ma_packet->timestamp[3] =  clock_cycle_count & 0xFF;
      }
      if (!err) {
        logger_write_entry('B', sizeof(MMC5983MAPacket_t));
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
  DataPacket *packet = (DataPacket *)&data_packet.data;
  TaskCommandOption cmd_status;
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
        xQueueSend(system_request_queue, &(SystemRequest){SYSREQ_FINISH_SETUP}, 0);
      }
    }
    vTaskDelay(1000);
    float dt = (float)packet->timestamp_seconds - last_time;
    // ukf_predict(&ukf, dt);
    // float measurement[10];
    // memcpy(measurement, &packet->pressure_pascals, sizeof(measurement));
    // ukf_update(&ukf, measurement);
    // memcpy(&packet->est_position_x_meters, ukf.X, UKF_STATE_DIMENSION * 4);
    last_time += dt;
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
    // xQueueSend(transmit_queue, &data_packet, 0);
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
      //CDC_Transmit_FS((uint8_t*)&packet, serialized_packet_len);
      // optionally transmit over uart if the setting is enabled
      if (firmSettings.uart_transfer_enabled && uart_tx_done) {
        uart_tx_done = false;
        HAL_UART_Transmit_DMA(firm_huart1, (uint8_t*)&packet, serialized_packet_len);
      }
    }
  }
}

void usb_read_data(void *argument) {
  uint8_t header_bytes[sizeof(data_packet.header)] = {
    0x11,
    0x11,
  };
  uint32_t payload_length;
  uint8_t received_bytes[COMMAND_READ_CHUNK_SIZE_BYTES];

  for (;;) {
    // Receive incoming USB data, attempt to parse header
    header_bytes[0] = header_bytes[1];
    xStreamBufferReceive(usb_rx_stream, &(header_bytes[1]), 1, portMAX_DELAY);
    // check if the 2 header bytes are valid, skip if invalid
    uint16_t header = header_bytes[0] | ((uint16_t)header_bytes[1] << 8);
    uint16_t msg_id = validate_message_header(header);
    if (msg_id == MSGID_INVALID) {
      continue;
    } 
    // reset header bytes
    header_bytes[1] = 0x11;
    // valid header, so get the identifier
    uint16_t identifier;
    xStreamBufferReceive(usb_rx_stream, (uint8_t *)&identifier, 2, portMAX_DELAY);
    msg_id = validate_message_identifier(header, identifier);

    // parse length field and get payload + crc bytes
    xStreamBufferReceive(usb_rx_stream, &payload_length, sizeof(payload_length), portMAX_DELAY);
    if (payload_length > COMMAND_READ_CHUNK_SIZE_BYTES - 2)
      continue;

    uint16_t bytes_read = xStreamBufferReceive(usb_rx_stream, received_bytes, payload_length + 2, portMAX_DELAY);
    if (bytes_read != payload_length + 2)
      led_set_status(MMC5983MA_FAIL);
    CDC_Transmit_FS((uint8_t *)&bytes_read, 2);
    // verify the data in the packet is valid by checking the crc across header, length, and payload
    if (!validate_message_crc16(header, identifier, payload_length, received_bytes)) {
      continue;
    }
    led_set_status(SD_CARD_FAIL);

    switch (msg_id) {
      case MSGID_MOCK_PACKET: {
        // create and fill a mock packet with data, and send to queue
        SensorPacket mock_packet;
        MockPacketID mock_type = process_mock_packet(identifier, payload_length, received_bytes, (uint8_t*)&mock_packet);
        switch (mock_type) {
          case MOCKID_BMP581:
            xQueueSend(mock_bmp581_queue, &mock_packet, portMAX_DELAY);
            break;
          case MOCKID_ICM45686:
            xQueueSend(mock_icm45686_queue, &mock_packet, portMAX_DELAY);
            break;
          case MOCKID_MMC5983MA:
            xQueueSend(mock_mmc5983ma_queue, &mock_packet, portMAX_DELAY);
            break;
          case MOCKID_SETTINGS: {
            // Process mock settings/calibration header and append to current log file
            FIRMSettings_t mock_firm_settings;
            CalibrationSettings_t mock_calibration_settings;
            HeaderFields header_fields;
            
            if (process_mock_settings_packet(received_bytes, payload_length, &mock_firm_settings, &mock_calibration_settings, &header_fields)) {
              // Successfully processed, append mock header to current log file
              logger_append_mock_header(&mock_firm_settings, &mock_calibration_settings, &header_fields);
              
            }
            logger_append_mock_header(&mock_firm_settings, &mock_calibration_settings, &header_fields);
            break;
          }
          default:
            break;
        }
        break;
      }
      case MSGID_COMMAND_PACKET: {
        Packet response;
        uint16_t response_header_and_id[2];
        message_get_response_id(header, identifier, response_header_and_id);
        response.header = response_header_and_id[0];
        response.identifier = response_header_and_id[1];
        response.packet_len = execute_command(identifier, received_bytes, payload_length, (ResponsePacket *)&response.data);
        xQueueSend(transmit_queue, &response, portMAX_DELAY);
        break;
      }
      case MSGID_SYSTEM_MANAGER_REDIRECT:
        
        xQueueSend(system_request_queue, (SystemRequest *)&identifier, portMAX_DELAY);
        break;
      default:
        continue;
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
