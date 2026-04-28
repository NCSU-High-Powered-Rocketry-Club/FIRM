/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mode_indicator_task.h"
#include "sensor_task.h"
#include "filter_data_task.h"
#include "packetizer_task.h"
#include "transmit_task.h"
#include "usb_read_data_task.h"
#include "transmit_frame.h"
#include "led.h"
#include "logger.h"
#include "mocking_handler.h"
#include "sensor_manager.h"
#include "settings_manager.h"
#include "system_settings.h"
#include "system_state.h"
#include "targets.h"
#include "firm_v1_0.h"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "queue.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "task.h"

#include <bmp581.h>
#include <icm45686.h>
#include <mmc5983ma.h>
#include <adxl371.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MOCK_QUEUE_LENGTH 50U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for startupTask */
osThreadId_t startupTaskHandle;
const osThreadAttr_t startupTask_attributes = {
  .name = "startupTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void StartupTask(void *argument);

/* USER CODE BEGIN PFP */

static bool mock_count_try_take(void *context);
static bool mock_count_give(void *context);
static size_t mock_count_get_count(void *context);
static void mock_count_reset(void *context);
static void firm_rtos_init(void);
static void configure_mocking_injection(void);
static uint32_t firm_time_cyccnt(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

static uint32_t firm_time_cyccnt(void) {
  return DWT->CYCCNT;
}

static void configure_mocking_injection(void) {
  MockSensorTaskInjectFns_t inject_fns = {
      .set_time_fn = set_time_fn,
      .set_barometer_read_fn = set_barometer_read_fn,
      .set_imu_read_fn = set_imu_read_fn,
      .set_magnetometer_read_fn = set_magnetometer_read_fn,
      .set_high_g_read_fn = set_high_g_read_fn,
      .get_time_fn = sensor_manager_get_time_fn,
      .get_barometer_read_fn = sensor_manager_get_barometer_read_fn,
      .get_imu_read_fn = sensor_manager_get_imu_read_fn,
      .get_magnetometer_read_fn = sensor_manager_get_magnetometer_read_fn,
      .get_high_g_read_fn = sensor_manager_get_high_g_read_fn,
  };

  mocking_handler_configure_sensor_task_injection(&inject_fns);
}

static void firm_rtos_init(void) {
  usb_rx_stream =
      xStreamBufferCreate(USB_RX_STREAM_BUFFER_SIZE_BYTES, USB_RX_STREAM_TRIGGER_LEVEL_BYTES);
  transmit_queue = xQueueCreate(TRANSMIT_QUEUE_LENGTH, sizeof(TransmitFrame_t));
  sensor_event_group = xEventGroupCreate();
  sensor_collected_group = xEventGroupCreate();

  if (usb_rx_stream == NULL || transmit_queue == NULL || sensor_event_group == NULL ||
      sensor_collected_group == NULL) {
    Error_Handler();
  }

  memset(&latest_data_packet, 0, sizeof(latest_data_packet));

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

  configure_mocking_injection();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SDIO_SD_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  if (firm_init_hardware())
    Error_Handler();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  firm_rtos_init();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of startupTask */
  startupTaskHandle = osThreadNew(StartupTask, NULL, &startupTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  firm_mode_indicator_task_handle =
      osThreadNew(firm_mode_indicator_task, NULL, &modeIndicatorTask_attributes);
  sensor_task_handle = osThreadNew(sensor_task, NULL, &sensorTask_attributes);
  packetizer_task_handle = osThreadNew(packetizer_task, NULL, &packetizerTask_attributes);
  filter_data_task_handle = osThreadNew(filter_data_task, NULL, &filterDataTask_attributes);
  transmit_task_handle = osThreadNew(transmit_data, NULL, &transmitTask_attributes);
  usb_read_task_handle = osThreadNew(usb_read_data, NULL, &usbReadTask_attributes);

  if (firm_mode_indicator_task_handle == NULL || sensor_task_handle == NULL ||
      filter_data_task_handle == NULL || packetizer_task_handle == NULL ||
      transmit_task_handle == NULL || usb_read_task_handle == NULL) {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Error_Handler();
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|DEBUG0_Pin|BMP581_CS_Pin|FLASH_CS_Pin
                          |DEBUG2_Pin|MMC5983MA_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADXL371_CS_GPIO_Port, ADXL371_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DEBUG1_Pin|ICM45686_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 DEBUG0_Pin FLASH_CS_Pin DEBUG2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|DEBUG0_Pin|FLASH_CS_Pin|DEBUG2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BMP581_CS_Pin MMC5983MA_CS_Pin */
  GPIO_InitStruct.Pin = BMP581_CS_Pin|MMC5983MA_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BMP581_Interrupt_Pin */
  GPIO_InitStruct.Pin = BMP581_Interrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BMP581_Interrupt_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CONF_CHECK_Pin */
  GPIO_InitStruct.Pin = CONF_CHECK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CONF_CHECK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ICM45686_Interrupt_Pin ADXL371_Interrupt_Pin */
  GPIO_InitStruct.Pin = ICM45686_Interrupt_Pin|ADXL371_Interrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MMC5983MA_Interrupt_Pin */
  GPIO_InitStruct.Pin = MMC5983MA_Interrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MMC5983MA_Interrupt_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ADXL371_CS_Pin */
  GPIO_InitStruct.Pin = ADXL371_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADXL371_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG1_Pin ICM45686_CS_Pin */
  GPIO_InitStruct.Pin = DEBUG1_Pin|ICM45686_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief ISR for interrupt pins
 * @note all ISR signals use this function, must check which pin before setting data ready flags
 * @param GPIO_Pin the EXTI pin that triggered the interrupt signal
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (sensor_event_group == NULL) {
    return;
  }

  if (GPIO_Pin == BMP581_Interrupt_Pin) {
    (void)xEventGroupSetBitsFromISR(sensor_event_group, SENSOR_EVENT_BAROMETER_READY,
                                    &xHigherPriorityTaskWoken);
  }
  if (GPIO_Pin == ICM45686_Interrupt_Pin) {
    (void)xEventGroupSetBitsFromISR(sensor_event_group, SENSOR_EVENT_IMU_READY,
                                    &xHigherPriorityTaskWoken);
  }
  if (GPIO_Pin == MMC5983MA_Interrupt_Pin) {
    (void)xEventGroupSetBitsFromISR(sensor_event_group, SENSOR_EVENT_MAGNETOMETER_READY,
                                    &xHigherPriorityTaskWoken);
  }
  if (GPIO_Pin == ADXL371_Interrupt_Pin) {
    (void)xEventGroupSetBitsFromISR(sensor_event_group, SENSOR_EVENT_HIGH_G_READY,
                                    &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  vTaskDelete(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartupTask */
/**
 * @brief Function implementing the startupTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartupTask */
void StartupTask(void *argument)
{
  /* USER CODE BEGIN StartupTask */

  system_state_set(SYSTEM_STATE_BOOT);

  // We use DWT (Data Watchpoint and Trace unit) to get a high resolution free-running timer.
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  set_time_fn(firm_time_cyccnt);

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

  if (icm45686_init()) {
    led_set_status(IMU_FAIL);
    Error_Handler();
  }

  if (mmc5983ma_init()) {
    led_set_status(MMC5983MA_FAIL);
    Error_Handler();
  }

  if (bmp581_init()) {
    led_set_status(BMP581_FAIL);
    Error_Handler();
  }

  if (adxl371_init()) {
    led_set_status(HIGH_G_FAIL);
    Error_Handler();
  }
  
  // set up the settings manager
  if (settings_manager_init()) {
    led_set_status(FLASH_CHIP_FAIL);
    Error_Handler();
  }

  // setup logger with the sensor data sizes
  logger_set_sensor_info(sizeof(BMP581RawData_t), sizeof(ICM45686RawData_t),
                         sizeof(MMC5983MARawData_t), sizeof(ADXL371RawData_t));

  // Open a new log and emit the metadata header.
  if (create_log()) {
    Error_Handler();
  }
  if (logger_write_header(*get_settings())) {
    Error_Handler();
  }

  // re-enable ISR's so that interrupts can trigger the sensor tasks to run
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  vTaskDelete(NULL);
  /* USER CODE END StartupTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
