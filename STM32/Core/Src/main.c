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
#define ENABLE_MAGNETOMETER 0
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
/* Use a small double-buffer so the main loop can prepare a new packet
  while a master is reading the current one. We avoid overwriting the
  buffer currently being transmitted. */
volatile uint8_t i2c2_tx_buf[2][128];
volatile uint8_t i2c2_active_buf = 0; /* index of buffer currently published */
volatile uint16_t i2c2_tx_len = 0;
volatile uint8_t i2c2_tx_ready = 0; /* set when active buffer contains valid data */
volatile uint8_t i2c2_tx_pending = 0; /* set when an updated packet is waiting to be swapped in */
volatile uint8_t i2c2_transmitting = 0; /* non-zero while a master read is in progress */
volatile uint8_t i2c2_rx_buf[32];
volatile uint8_t i2c2_rx_len = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
#if ENABLE_MAGNETOMETER
static void MX_I2C1_Init(void);
#endif
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
#if ENABLE_MAGNETOMETER
  MX_I2C1_Init();
#endif
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
    
#if ENABLE_MAGNETOMETER
    if (mmc5983ma_init(&hi2c1, 0x30)) {
        Error_Handler();
    }
#endif

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
#if ENABLE_MAGNETOMETER
  mmc5983ma_get_magnetic_field_scale_factor(),
#else
  0.0f,
#endif
    };
    

    logger_write_header(&header_fields);

#if ENABLE_MAGNETOMETER
    // incrementing value for magnetometer calibration
    uint8_t magnetometer_flip = 0;
#endif

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
#if ENABLE_MAGNETOMETER
    MMC5983MAPacket_t mag_packet;
    mmc5983ma_read_data(&mag_packet, &magnetometer_flip);
#endif
    
    /* Populate I2C2 buffer with initial packet so Pi can read immediately */
    usb_serialize_calibrated_packet(&calibrated_packet, &serialized_packet);
    
  // Manually pack the packet into the inactive buffer to avoid struct padding issues
  // Total: 2 (header) + 2 (length) + 52 (payload: 11 floats + 1 double) + 2 (crc) = 58 bytes
  uint8_t *buf = (uint8_t *)i2c2_tx_buf[1 - i2c2_active_buf];
  size_t offset = 0;
    
  // Pack header and length
  memcpy(buf + offset, &serialized_packet.header, 2);
  offset += 2;
  memcpy(buf + offset, &serialized_packet.length, 2);
  offset += 2;
    
  // Pack payload fields individually to avoid padding (11 floats + 1 double = 52 bytes)
  memcpy(buf + offset, &serialized_packet.payload.temperature, sizeof(float));
  offset += sizeof(float);
  memcpy(buf + offset, &serialized_packet.payload.pressure, sizeof(float));
  offset += sizeof(float);
  memcpy(buf + offset, &serialized_packet.payload.accel_x, sizeof(float));
  offset += sizeof(float);
  memcpy(buf + offset, &serialized_packet.payload.accel_y, sizeof(float));
  offset += sizeof(float);
  memcpy(buf + offset, &serialized_packet.payload.accel_z, sizeof(float));
  offset += sizeof(float);
  memcpy(buf + offset, &serialized_packet.payload.angular_rate_x, sizeof(float));
  offset += sizeof(float);
  memcpy(buf + offset, &serialized_packet.payload.angular_rate_y, sizeof(float));
  offset += sizeof(float);
  memcpy(buf + offset, &serialized_packet.payload.angular_rate_z, sizeof(float));
  offset += sizeof(float);
  memcpy(buf + offset, &serialized_packet.payload.magnetic_field_x, sizeof(float));
  offset += sizeof(float);
  memcpy(buf + offset, &serialized_packet.payload.magnetic_field_y, sizeof(float));
  offset += sizeof(float);
  memcpy(buf + offset, &serialized_packet.payload.magnetic_field_z, sizeof(float));
  offset += sizeof(float);
  memcpy(buf + offset, &serialized_packet.payload.timestamp_sec, sizeof(double));
  offset += sizeof(double);
    
  // Calculate CRC over the packed buffer (without the CRC bytes)
  uint16_t crc = crc16_ccitt(buf, 56);  // 56 = header(2) + length(2) + payload(52)
  memcpy(buf + offset, &crc, 2);
    
  /* Publish into the active buffer since no transfer is in progress yet */
  i2c2_tx_len = 58;
  /* swap inactive -> active */
  i2c2_active_buf = 1 - i2c2_active_buf;
  i2c2_tx_ready = 1;
  i2c2_tx_pending = 0;
  i2c2_transmitting = 0;
    
    /* Enable I2C2 listen mode now that I2C1 master initialization is complete */
    HAL_I2C_EnableListen_IT(&hi2c2);
    char debug_msg[64];
    snprintf(debug_msg, sizeof(debug_msg), "I2C2: Init CRC=0x%04X, offset=%u", crc, (unsigned)offset);
    serialPrintStr(debug_msg);
    
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
#if ENABLE_MAGNETOMETER
        if (mmc5983ma_has_new_data) {
            MMC5983MAPacket_t* mmc5983ma_packet = logger_malloc_packet(sizeof(MMC5983MAPacket_t));
            if (!mmc5983ma_read_data(mmc5983ma_packet, &magnetometer_flip)) {
                mmc5983ma_has_new_data = false;
                logger_write_entry('M', sizeof(MMC5983MAPacket_t));
                mmc5983ma_convert_packet(mmc5983ma_packet, &calibrated_packet);
                any_new_data_collected = true;
            }
        }
#endif

        // Whenever new sensor data is collected, update the I2C2 buffer for the Pi to read
        if (any_new_data_collected) {
            usb_serialize_calibrated_packet(&calibrated_packet, &serialized_packet);
            
      // Manually pack the packet into the inactive buffer to avoid races with
      // an ongoing I2C read. If a master is currently reading, we mark the
      // prepared packet as pending and it will be swapped in when the
      // transfer completes.
      uint8_t inactive = 1 - i2c2_active_buf;
      uint8_t *buf = (uint8_t *)i2c2_tx_buf[inactive];
            size_t offset = 0;
            
            // Pack header and length
            memcpy(buf + offset, &serialized_packet.header, 2);
            offset += 2;
            memcpy(buf + offset, &serialized_packet.length, 2);
            offset += 2;
            
            // Pack payload fields individually to avoid padding (11 floats + 1 double = 52 bytes)
            memcpy(buf + offset, &serialized_packet.payload.temperature, sizeof(float));
            offset += sizeof(float);
            memcpy(buf + offset, &serialized_packet.payload.pressure, sizeof(float));
            offset += sizeof(float);
            memcpy(buf + offset, &serialized_packet.payload.accel_x, sizeof(float));
            offset += sizeof(float);
            memcpy(buf + offset, &serialized_packet.payload.accel_y, sizeof(float));
            offset += sizeof(float);
            memcpy(buf + offset, &serialized_packet.payload.accel_z, sizeof(float));
            offset += sizeof(float);
            memcpy(buf + offset, &serialized_packet.payload.angular_rate_x, sizeof(float));
            offset += sizeof(float);
            memcpy(buf + offset, &serialized_packet.payload.angular_rate_y, sizeof(float));
            offset += sizeof(float);
            memcpy(buf + offset, &serialized_packet.payload.angular_rate_z, sizeof(float));
            offset += sizeof(float);
            memcpy(buf + offset, &serialized_packet.payload.magnetic_field_x, sizeof(float));
            offset += sizeof(float);
            memcpy(buf + offset, &serialized_packet.payload.magnetic_field_y, sizeof(float));
            offset += sizeof(float);
            memcpy(buf + offset, &serialized_packet.payload.magnetic_field_z, sizeof(float));
            offset += sizeof(float);
            memcpy(buf + offset, &serialized_packet.payload.timestamp_sec, sizeof(double));
            offset += sizeof(double);
            
            // Calculate CRC over the packed buffer (without the CRC bytes)
      uint16_t crc = crc16_ccitt(buf, 56);  // 56 = header(2) + length(2) + payload(52)
      memcpy(buf + offset, &crc, 2);

      /* If no transfer in progress, swap the buffers immediately and publish.
         Otherwise mark pending and let the TxCplt callback perform the swap. */
      if (!i2c2_transmitting) {
        i2c2_tx_len = 58;
        i2c2_active_buf = inactive;
        i2c2_tx_ready = 1;
        i2c2_tx_pending = 0;
      } else {
        /* queue the prepared buffer to be swapped in after the current read */
        i2c2_tx_len = 58; /* still store the length for the pending buffer */
        i2c2_tx_pending = 1;
      }
            
            // If USB serial communication setting is enabled, also transmit via USB
            if (firmSettings.serial_transfer_enabled) {
                usb_transmit_serialized_packet(&serialized_packet);
            }
            
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
#if ENABLE_MAGNETOMETER
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
#endif

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

#if !ENABLE_MAGNETOMETER
  /* Keep I2C1 pins pulled up while peripheral is disabled */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif

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
    /* Master will write to us */
    i2c2_rx_len = 0;
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, (uint8_t *)i2c2_rx_buf, sizeof(i2c2_rx_buf), I2C_FIRST_AND_LAST_FRAME);
  } else {
    /* Master will read from us */
    if (i2c2_tx_ready && i2c2_tx_len > 0) {
      /* mark transmitting so the main loop won't swap the active buffer */
      i2c2_transmitting = 1;
      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t *)i2c2_tx_buf[i2c2_active_buf], i2c2_tx_len, I2C_FIRST_AND_LAST_FRAME);
    } else {
      /* Send zero byte if no data ready */
      static uint8_t zero = 0x00;
      i2c2_transmitting = 1;
      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &zero, 1, I2C_FIRST_AND_LAST_FRAME);
    }
  }
}

/* Slave transmit complete */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C2) return;
  /* transfer finished; clear transmitting flag and swap in any pending update */
  i2c2_transmitting = 0;
  if (i2c2_tx_pending) {
    /* swap active -> the buffer prepared earlier */
    i2c2_active_buf = 1 - i2c2_active_buf;
    i2c2_tx_pending = 0;
    i2c2_tx_ready = 1;
  }
  i2c2_enable_listen();
}

/* Slave receive complete */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C2) return;
  i2c2_rx_len = hi2c->XferCount;
  i2c2_enable_listen();
}

/* Error callback */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C2) return;
  
  /* Clear all error flags and re-enable listen */
  __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR);
  /* AF (NACK) is a normal condition when a master finishes reading; clearing
     it is fine but ensure we also exit transmitting state so pending data can
     be published. */
  __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);
  i2c2_transmitting = 0;
  if (i2c2_tx_pending) {
    i2c2_active_buf = 1 - i2c2_active_buf;
    i2c2_tx_pending = 0;
    i2c2_tx_ready = 1;
  }
  i2c2_enable_listen();
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
