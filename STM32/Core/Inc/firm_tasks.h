#ifndef FIRM_TASKS_H
#define FIRM_TASKS_H

#include <bmp581.h>
#include <icm45686.h>
#include <mmc5983ma.h>
#include "data_processing/preprocessor.h"
#include "logger.h"
#include "usb_serializer.h"
#include "led.h"
#include "settings.h"

#include "cmsis_os.h"

#define FIRM_TASK_DEFAULT_PRIORITY 100
#define BMP581_POLL_RATE_HZ 500
#define ICM45686_POLL_RATE_HZ 800
#define MMC5983MA_POLL_RATE_HZ 200

#define MAX_WAIT_TIME(hz) (TickType_t)(pdMS_TO_TICKS(1000 / (hz)) + 1)

extern osThreadId_t bmp581_task_handle;
extern const osThreadAttr_t bmp581Task_attributes;
extern osThreadId_t icm45686_task_handle;
extern const osThreadAttr_t icm45686Task_attributes;
extern osThreadId_t mmc5983ma_task_handle;
extern const osThreadAttr_t mmc5983maTask_attributes;
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

void collect_bmp581_data_task(void *argument);
void collect_icm45686_data_task(void *argument);
void collect_mmc5983ma_data_task(void *argument);

void TaskWriteSerialData(void *argument);

#endif // FIRM_TASKS_H