#pragma once

#include <bmp581.h>
#include <icm45686.h>
#include <mmc5983ma.h>
#include "data_processing/preprocessor.h"
#include "logger.h"
#include "usb_serializer.h"
#include "led.h"
#include "settings.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "queue.h"
#include "task.h"
#include "commands.h"

#define FIRM_TASK_DEFAULT_PRIORITY 100
#define BMP581_POLL_RATE_HZ 500
#define ICM45686_POLL_RATE_HZ 800
#define MMC5983MA_POLL_RATE_HZ 200
#define TRANSMIT_FREQUENCY_HZ 10

#define MAX_WAIT_TIME(hz) (TickType_t)(pdMS_TO_TICKS(1000 / (hz)) + 1)

#define USB_RX_STREAM_BUFFER_SIZE_BYTES 512
#define USB_RX_STREAM_TRIGGER_LEVEL_BYTES 1

#define COMMAND_QUEUE_LENGTH 5
#define RESPONSE_QUEUE_LENGTH 5

#define COMMAND_RX_READ_CHUNK_SIZE_BYTES 64

#define COMMAND_PAYLOAD_MAX_LEN_BYTES 56
#define COMMAND_RESPONSE_PACKET_SIZE_BYTES 66

extern osThreadId_t bmp581_task_handle;
extern osThreadId_t mmc5983ma_task_handle;
extern osThreadId_t icm45686_task_handle;
extern osThreadId_t usb_transmit_task_handle;
extern osThreadId_t uart_transmit_task_handle;
extern osThreadId_t usb_read_task_handle;
extern osThreadId_t command_handler_task_handle;

extern StreamBufferHandle_t usb_rx_stream;
extern QueueHandle_t command_queue;
extern QueueHandle_t response_queue;

extern const osThreadAttr_t bmp581Task_attributes;
extern const osThreadAttr_t mmc5983maTask_attributes;
extern const osThreadAttr_t icm45686Task_attributes;
extern const osThreadAttr_t usbTask_attributes;
extern const osThreadAttr_t uartTask_attributes;
extern const osThreadAttr_t usbReadTask_attributes;
extern const osThreadAttr_t commandHandlerTask_attributes;

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

/**
 * @brief Initializes firm, including all sensors and the logger
 *
 * @param spi_handles struct containing all SPI handles, from the HAL
 * @param i2c_handles struct containing all I2C handles, from the HAL
 * @param dma_handles struct containing all DMA handles, from the HAL
 * @param uart_handles struct containing all UART handles, from the HAL
 * @retval Whether FIRM successfully initialized, 0 on successful write
 */
int initialize_firm(SPIHandles *spi_handles, I2CHandles *i2c_handles, DMAHandles *dma_handles,
                    UARTHandles *uart_handles);

/**
 * @brief Initializes FreeRTOS objects (queues, streams)
 */
void firm_rtos_init(void);

/**
 * Callback function to handle received USB data. It is called from the USB ISR.
 * 
 * @param buffer Pointer to the received data buffer
 * @param data_length Length of the received data
 */
void usb_receive_callback(uint8_t *buffer, uint32_t data_length);

void collect_bmp581_data_task(void *argument);
void collect_icm45686_data_task(void *argument);
void collect_mmc5983ma_data_task(void *argument);
void usb_transmit_data(void *argument);
void uart_transmit_data(void *argument);
void usb_read_data(void *argument);
void command_handler_task(void *argument);
