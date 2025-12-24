#include "firm_tasks.h"
#include "usb_print_debug.h"
#include <string.h>

// task handles
osThreadId_t bmp581_task_handle;
osThreadId_t icm45686_task_handle;
osThreadId_t mmc5983ma_task_handle;
osThreadId_t usb_transmit_task_handle;
osThreadId_t uart_transmit_task_handle;
osThreadId_t usb_read_task_handle;
osThreadId_t command_handler_task_handle;

StreamBufferHandle_t usb_rx_stream;
QueueHandle_t command_queue;

// task attributes
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
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};
const osThreadAttr_t uartTask_attributes = {
    .name = "uartTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};

// mutexes
osMutexId_t sensorDataMutexHandle;
const osMutexAttr_t sensorDataMutex_attributes = {
  .name = "sensorDataMutex"
};

// instance of the calibrated data packet from the preprocessor to be reused
CalibratedDataPacket_t calibrated_packet = {0};

// instance of the serialized packet, will be reused
SerializedPacket_t serialized_packet = {0};

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
  serializer_init_packet(&serialized_packet); // initializes the packet length and header bytes
  

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
  // 512 bytes buffer, trigger level 1 (wake up on every byte if needed, or optimize later)
  usb_rx_stream = xStreamBufferCreate(512, 1);
  // Queue for parsed commands
  command_queue = xQueueCreate(5, sizeof(Command_t));
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
        bmp581_convert_packet(bmp581_packet, &calibrated_packet);
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
        icm45686_convert_packet(icm45686_packet, &calibrated_packet);
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
        mmc5983ma_convert_packet(mmc5983ma_packet, &calibrated_packet);
      }
      osMutexRelease(sensorDataMutexHandle);
    } else {
      // TODO: error handling
    }
  }
}

void usb_transmit_data(void *argument) {
  
  const TickType_t transmit_freq = MAX_WAIT_TIME(TRANSMIT_FREQUENCY_HZ);
  TickType_t lastWakeTime = xTaskGetTickCount();
  
  for (;;) {
    if (firmSettings.usb_transfer_enabled) {
      osMutexAcquire(sensorDataMutexHandle, osWaitForever);
      serialize_calibrated_packet(&calibrated_packet, &serialized_packet);
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
      serialize_calibrated_packet(&calibrated_packet, &serialized_packet);
      osMutexRelease(sensorDataMutexHandle);
      if (uart_tx_done) {
        uart_tx_done = false;
        HAL_UART_Transmit_DMA(firm_huart1, (uint8_t*)&serialized_packet, (uint16_t)sizeof(SerializedPacket_t));
      }
    }
    vTaskDelayUntil(&lastWakeTime, transmit_freq);
  }
}
void usb_read_data(void *argument) {
  const size_t PACKET_SIZE = 64;
  uint8_t packet_buffer[64];
  size_t bytes_received = 0;

  for (;;) {
    // Read until we have a full packet
    // We request the remaining bytes needed to fill the 64-byte buffer
    size_t received = xStreamBufferReceive(usb_rx_stream, 
                                           &packet_buffer[bytes_received], 
                                           PACKET_SIZE - bytes_received, 
                                           portMAX_DELAY);
    bytes_received += received;

    if (bytes_received >= PACKET_SIZE) {
      Command_t cmd;
      if (parse_command(packet_buffer, PACKET_SIZE, &cmd)) {
        // Check for Cancel Command
        if (cmd.id == CMD_CANCEL_ID) {
          if (command_handler_task_handle != NULL) {
            xTaskNotify(command_handler_task_handle, 0x01, eSetBits);
          }
        } else {
          xQueueSend(command_queue, &cmd, 0);
        }
      }

      // Reset for next packet
      bytes_received = 0;
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == firm_huart1)
    uart_tx_done = true;
}