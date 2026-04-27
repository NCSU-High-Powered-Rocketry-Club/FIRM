#pragma once

#include "main.h"
#include "packets.h"
#include "transmit_frame.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "event_groups.h"
#include "queue.h"
#include "stream_buffer.h"

#include <stdbool.h>
#include <stdint.h>

#define MAX_WAIT_TIME(hz) (TickType_t)(pdMS_TO_TICKS(1000 / (hz)) + 1)

#define USB_RX_STREAM_BUFFER_SIZE_BYTES 6144
#define USB_RX_STREAM_TRIGGER_LEVEL_BYTES 1

#define SYSTEM_REQUEST_QUEUE_LENGTH 5
#define TRANSMIT_QUEUE_LENGTH 10
#define MOCK_QUEUE_LENGTH 50

// Task-notification bits for sensor tasks.
#define SENSOR_NOTIFY_ISR_BIT (1UL << 0)
#define SENSOR_NOTIFY_MOCK_BIT (1UL << 1)

extern StreamBufferHandle_t usb_rx_stream;
extern QueueHandle_t transmit_queue;
extern QueueHandle_t system_request_queue;

extern EventGroupHandle_t sensors_collected;
extern DataPacket_t latest_data_packet;

extern osMutexId_t sensorDataMutexHandle;
extern const osMutexAttr_t sensorDataMutex_attributes;

/**
 * Struct to contain all SPI handles for the firm initialization function
 */
typedef struct {
  SPI_HandleTypeDef *hspi1;
  SPI_HandleTypeDef *hspi2;
  SPI_HandleTypeDef *hspi3;
} SPIHandles;

/**
 * Struct to contain all I2C handles for the firm initialization function
 */
typedef struct {
  I2C_HandleTypeDef *hi2c1;
  I2C_HandleTypeDef *hi2c2;
} I2CHandles;

/**
 * Struct to contain all DMA handles for the firm initialization function
 */
typedef struct {
  DMA_HandleTypeDef *hdma_sdio_rx;
  DMA_HandleTypeDef *hdma_sdio_tx;
} DMAHandles;

/**
 * Struct to contain all UART handles for the firm initialization function
 */
typedef struct {
  UART_HandleTypeDef *huart1;
} UARTHandles;

int initialize_firm(SPIHandles *spi_handles, I2CHandles *i2c_handles, DMAHandles *dma_handles,
                    UARTHandles *uart_handles);

void firm_rtos_init(void);
