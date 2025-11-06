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

#include <bmp581.h>
#include <icm45686.h>
#include "logger.h"
#include <mmc5983ma.h>
#include "data_processing/preprocessor.h"
#include "usb_serializer.h"
#include "settings.h"
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "usb_print_debug.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

/* USER CODE BEGIN PV */

/* I2C2 runtime state shared across callbacks */
volatile uint8_t i2c2_tx_buf[128];
volatile uint16_t i2c2_tx_len = 0;
volatile uint8_t i2c2_tx_ready = 0;
volatile uint8_t i2c2_rx_buf[32];
volatile uint8_t i2c2_pending_addr = 0;
volatile bool i2c2_recover_request = false;
volatile uint16_t i2c2_tx_index = 0; /* byte index for multi-byte slave transmit */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// These flags can be changed at any time from the interrupts. When they are set
// to true, it means that the corresponding sensor has new data ready to be read.
volatile bool bmp581_has_new_data = false;
volatile bool icm45686_has_new_data = false;
volatile bool mmc5983ma_has_new_data = false;
// number of times the DWT timestamp has overflowed. This happens every ~25 seconds
volatile uint32_t dwt_overflow_count = 0;

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

  /* USER NOTE: do a full I2C2 peripheral reset and release the bus lines
    before initializing peripherals. This prevents the MCU from holding SCL
    low if the peripheral was left in a bad state (clock-stretching forever). */
  serialPrintStr("I2C2: performing peripheral reset and bus release");
  HAL_I2C_DeInit(&hi2c2);
  __HAL_RCC_I2C2_FORCE_RESET();
  HAL_Delay(10);
  __HAL_RCC_I2C2_RELEASE_RESET();
  HAL_Delay(10);

  /* Ensure GPIOB clock enabled and force SCL/SDA high using OD outputs so
    any stuck low is released and pull-ups can recharge. */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_SET);
  {
   GPIO_InitTypeDef GPIO_InitStruct = {0};
   GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; /* drive high but open-drain */
   GPIO_InitStruct.Pull = GPIO_PULLUP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  HAL_Delay(100); /* allow bus to settle with pull-ups */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

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
    HAL_Delay(500); // purely for debug purposes, allows time to connect to USB serial terminal

    if (icm45686_init(&hspi2, GPIOB, GPIO_PIN_9)) {
        Error_Handler();
    }

    if (bmp581_init(&hspi2, GPIOC, GPIO_PIN_2)) {
        Error_Handler();
    }
    
    if (mmc5983ma_init(&hi2c1, 0x30)) {
        Error_Handler();
    }

    // set up settings module with flash chip
    if (settings_init(&hspi1, GPIOC, GPIO_PIN_4)) {
        Error_Handler();
    }

    // Setup the SD card
    FRESULT res = logger_init(&hdma_sdio_tx);
    if (res) {
        serialPrintStr("Failed to initialized the logger (SD card)");
        Error_Handler();
    }
    
    // get scale factor values for each sensor to put in header
    HeaderFields header_fields = {
        bmp581_get_temp_scale_factor(),
        bmp581_get_pressure_scale_factor(),
        icm45686_get_accel_scale_factor(),
        icm45686_get_gyro_scale_factor(),
        mmc5983ma_get_magnetic_field_scale_factor(),
    };
    

    logger_write_header(&header_fields);

    // incrementing value for magnetometer calibration
    uint8_t magnetometer_flip = 0;

    // Toggle LED:
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);

    

    // instance of the calibrated data packet from the preprocessor to be reused
    CalibratedDataPacket_t calibrated_packet = {0};
    // instance of the serialized packet, will be reused
    SerializedPacket_t serialized_packet = {0};
    serializer_init_packet(&serialized_packet); // initializes the packet length and header bytes
    
    // check to verify if any new data has been collected, from any of the sensors
    bool any_new_data_collected = false;

    // the IMU runs into issues when the fifo is full at the very beginning, causing the interrupt
    // to be pulled back low too fast, and the ISR doesn't catch it for whatever reason. Doing
    // this initial read will prevent that.
    ICM45686Packet_t imu_packet;
    icm45686_read_data(&imu_packet);
    MMC5983MAPacket_t mag_packet;
    mmc5983ma_read_data(&mag_packet, &magnetometer_flip);
    
    /* Populate I2C2 buffer with initial packet so Pi can read immediately */
    usb_serialize_calibrated_packet(&calibrated_packet, &serialized_packet);
    memcpy((void*)i2c2_tx_buf, (const void*)&serialized_packet, sizeof(SerializedPacket_t));
    i2c2_tx_len = (uint16_t)sizeof(SerializedPacket_t);
    i2c2_tx_ready = 1;
    
    /* Enable I2C2 listen mode now that I2C1 master initialization is complete */
    HAL_I2C_EnableListen_IT(&hi2c2);
    serialPrintStr("I2C2: Listen enabled with initial packet ready");
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    // This is the main loop. It's constantly checking to see if any of the sensors have
    // new data to read, and if so, logs it.
    while (1) {
        if (bmp581_has_new_data) {
            BMP581Packet_t* bmp581_packet = logger_malloc_packet(sizeof(BMP581Packet_t));
    	    if (!bmp581_read_data(bmp581_packet)) {
    	        bmp581_has_new_data = false;
    	        logger_write_entry('B', sizeof(BMP581Packet_t));
                bmp581_convert_packet(bmp581_packet, &calibrated_packet);
                any_new_data_collected = true;
    	    }
    	}

    	if (icm45686_has_new_data) {
    	    ICM45686Packet_t* icm45686_packet = logger_malloc_packet(sizeof(ICM45686Packet_t)); 
    	    if (!icm45686_read_data(icm45686_packet)) {
    	        icm45686_has_new_data = false;
    	        logger_write_entry('I', sizeof(ICM45686Packet_t));
                icm45686_convert_packet(icm45686_packet, &calibrated_packet);
                any_new_data_collected = true;
    	    }
    	}
    	if (mmc5983ma_has_new_data) {
    	    MMC5983MAPacket_t* mmc5983ma_packet = logger_malloc_packet(sizeof(MMC5983MAPacket_t));
    	    if (!mmc5983ma_read_data(mmc5983ma_packet, &magnetometer_flip)) {
    	        mmc5983ma_has_new_data = false;
    	        logger_write_entry('M', sizeof(MMC5983MAPacket_t));
                mmc5983ma_convert_packet(mmc5983ma_packet, &calibrated_packet);
                any_new_data_collected = true;
    	    }
    	}

        // if USB serial communication setting is enabled, and new data is collected, serialize
        // and transmit it
        if (firmSettings.serial_transfer_enabled && any_new_data_collected) {
            usb_serialize_calibrated_packet(&calibrated_packet, &serialized_packet);
            usb_transmit_serialized_packet(&serialized_packet);
            
            // Also populate I2C2 TX buffer so Pi Zero can read the latest packet via I2C.
            // Copy the serialized packet atomically to avoid race with I2C ISR.
            __disable_irq();
            memcpy((void*)i2c2_tx_buf, (const void*)&serialized_packet, sizeof(SerializedPacket_t));
            i2c2_tx_len = (uint16_t)sizeof(SerializedPacket_t);
            i2c2_tx_ready = 1;
            __enable_irq();
            
            any_new_data_collected = false;
        }

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
  /* Use 0x48 (7-bit) slave address. HAL expects the 7-bit address left-shifted
    in some HAL versions, so use 0x48 << 1 == 0x90 (144). This matches existing
    tooling that used 144 for a 7-bit address of 0x48. */
  hi2c2.Init.OwnAddress1 = 144;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|Feather_LED_Pin|BMP581_CS_Pin|FLASH_CS_Pin
                          |DEBUG2_Pin|MMC5983MA_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DEBUG0_Pin|DEBUG1_Pin|ICM45686_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 Feather_LED_Pin BMP581_CS_Pin FLASH_CS_Pin
                           DEBUG2_Pin MMC5983MA_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|Feather_LED_Pin|BMP581_CS_Pin|FLASH_CS_Pin
                          |DEBUG2_Pin|MMC5983MA_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BMP581_Interrupt_Pin ICM45686_Interrupt_Pin */
  GPIO_InitStruct.Pin = BMP581_Interrupt_Pin|ICM45686_Interrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CONF_CHECK_Pin */
  GPIO_InitStruct.Pin = CONF_CHECK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CONF_CHECK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG0_Pin DEBUG1_Pin ICM45686_CS_Pin */
  GPIO_InitStruct.Pin = DEBUG0_Pin|DEBUG1_Pin|ICM45686_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
    if (GPIO_Pin == BMP581_Interrupt_Pin) {
        bmp581_has_new_data  = true;
    }
    if (GPIO_Pin == ICM45686_Interrupt_Pin) {
        icm45686_has_new_data  = true;
    }
    if (GPIO_Pin == MMC5983MA_Interrupt_Pin) {
        mmc5983ma_has_new_data = true;
    }
}

/* Helper to re-enable listen mode (safe to call from callbacks) */
static void i2c2_enable_listen(void) {
  HAL_I2C_EnableListen_IT(&hi2c2);
}

/* Address match callback: start sequence transmit or receive */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  if (hi2c->Instance != I2C2) return;

  if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
    /* Master will write to us: prepare receive buffer */
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, (uint8_t *)i2c2_rx_buf, sizeof(i2c2_rx_buf), I2C_FIRST_AND_LAST_FRAME);
    serialPrintStr("I2C2: Master->Slave receive started");
  } else {
    /* Master will read from us: transmit entire buffer at once */
    if (i2c2_tx_len == 0 || !i2c2_tx_ready) {
      /* fallback single zero byte - clear any stale state first */
      static uint8_t zero = 0x00;
      i2c2_tx_ready = 0;
      i2c2_tx_len = 0;
      i2c2_tx_index = 0;
      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &zero, 1, I2C_FIRST_AND_LAST_FRAME);
      serialPrintStr("I2C2: Slave send fallback 0x00 (single byte)");
    } else {
      /* Send entire buffer at once with FIRST_AND_LAST_FRAME.
         HAL will handle the byte-by-byte hardware transmission internally. */
      i2c2_tx_index = 0;
      char dbg[64];
      sprintf(dbg, "I2C2: AddrCb sending %d bytes (buffer mode)", (int)i2c2_tx_len);
      serialPrintStr(dbg);
      
      HAL_StatusTypeDef status = HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t *)i2c2_tx_buf, i2c2_tx_len, I2C_FIRST_AND_LAST_FRAME);
      sprintf(dbg, "I2C2: Transmit status=%d", status);
      serialPrintStr(dbg);
    }
  }
}

/* Slave transmit complete: re-enable listen but keep data available */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C2) return;

  /* Transfer complete: re-enable listen mode but keep buffer valid for next read.
     The buffer will be updated when new sensor data arrives in the main loop. */
  serialPrintStr("I2C2: Slave TX complete; re-enabling listen");
  i2c2_enable_listen();
}

/* Slave receive complete */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C2) return;
  serialPrintStr("I2C2: Slave RX complete");
  i2c2_enable_listen();
}

/* Error callback: handle NACK (AF) from master to stop TX sequence */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C2) return;

  if (hi2c->ErrorCode & HAL_I2C_ERROR_AF) {
    /* Master NACKed the last byte: end transmit and re-enable listen */
    i2c2_tx_ready = 0;
    i2c2_tx_len = 0;
    i2c2_tx_index = 0;
    serialPrintStr("I2C2: Master NACK (AF) - clearing TX state and re-enabling listen");
    i2c2_enable_listen();
  } else {
    /* other error: log and attempt to re-enable listen */
    serialPrintStr("I2C2: I2C Error (non-AF)");
    i2c2_enable_listen();
  }
}

/* Listen complete callback: re-enter listen if needed */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C2) return;
  /* Re-enable listen to be ready for next address match */
  HAL_I2C_EnableListen_IT(hi2c);
}
/* USER CODE END 4 */

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