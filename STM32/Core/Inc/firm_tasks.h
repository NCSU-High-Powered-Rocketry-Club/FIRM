#pragma once

#include "firm_fsm.h"
#include <bmp581.h>
#include <icm45686.h>
#include <mmc5983ma.h>
#include "data_preprocess.h"
#include "logger.h"
#include "utils.h"
#include "led.h"
#include "settings.h"
#include "unscented_kalman_filter.h"
#include "ukf_functions.h"
#include "messages.h"
#include "mocking_handler.h"


#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "queue.h"
#include "task.h"
#include "commands.h"

#define BMP581_POLL_RATE_HZ 500
#define ICM45686_POLL_RATE_HZ 800
#define MMC5983MA_POLL_RATE_HZ 225
#define TRANSMIT_FREQUENCY_HZ 100
#define KALMAN_FILTER_STARTUP_DELAY_TIME_MS 1000

#define MAX_WAIT_TIME(hz) (TickType_t)(pdMS_TO_TICKS(1000 / (hz)) + 1)

#define USB_RX_STREAM_BUFFER_SIZE_BYTES 6144
#define USB_RX_STREAM_TRIGGER_LEVEL_BYTES 1

#define SYSTEM_REQUEST_QUEUE_LENGTH 5
#define TRANSMIT_QUEUE_LENGTH 10
#define MOCK_QUEUE_LENGTH 50

extern osThreadId_t system_manager_task_handle;
extern osThreadId_t firm_mode_indicator_task_handle;
extern osThreadId_t bmp581_task_handle;
extern osThreadId_t icm45686_task_handle;
extern osThreadId_t mmc5983ma_task_handle;
extern osThreadId_t packetizer_task_handle;
extern osThreadId_t transmit_task_handle;
extern osThreadId_t usb_read_task_handle;
extern osThreadId_t filter_data_task_handle;
extern osThreadId_t mock_packet_handler_handle;

// Task-notification bits for sensor tasks.
// We must distinguish EXTI (live) wakeups from mock dispatch wakeups so that
// in mock mode the EXTI ISR cannot cause out-of-order ring pops.
#define SENSOR_NOTIFY_ISR_BIT  (1UL << 0)
#define SENSOR_NOTIFY_MOCK_BIT (1UL << 1)

extern QueueHandle_t system_request_queue;

extern const osThreadAttr_t systemManagerTask_attributes;
extern const osThreadAttr_t modeIndicatorTask_attributes;
extern const osThreadAttr_t bmp581Task_attributes;
extern const osThreadAttr_t icm45686Task_attributes;
extern const osThreadAttr_t mmc5983maTask_attributes;
extern const osThreadAttr_t packetizerTask_attributes;
extern const osThreadAttr_t transmitTask_attributes;
extern const osThreadAttr_t usbReadTask_attributes;
extern const osThreadAttr_t filterDataTask_attributes;
extern const osThreadAttr_t mockPacketTask_attributes;

extern osMutexId_t sensorDataMutexHandle;
extern const osMutexAttr_t sensorDataMutex_attributes;

typedef union {
  DataPacket data_packet;
  ResponsePacket response_packet;
} PacketPayload;

typedef struct {
  uint16_t header;
  uint16_t identifier;
  uint32_t packet_len;
  PacketPayload data;
  uint16_t crc;
} Packet;

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
void usb_receive_callback(const uint8_t *buffer, uint32_t data_length);

void system_manager_task(void *argument);
void firm_mode_indicator_task(void *argument);
void collect_bmp581_data_task(void *argument);
void collect_icm45686_data_task(void *argument);
void collect_mmc5983ma_data_task(void *argument);
void packetizer_task(void *argument);
void filter_data_task(void *argument);
void transmit_data(void *argument);
void usb_read_data(void *argument);
void mock_packet_handler(void *argument);
