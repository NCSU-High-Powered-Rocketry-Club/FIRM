/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Feather_LED_Pin GPIO_PIN_1
#define Feather_LED_GPIO_Port GPIOC
#define BMP581_CS_Pin GPIO_PIN_2
#define BMP581_CS_GPIO_Port GPIOC
#define BMP581_Interrupt_Pin GPIO_PIN_3
#define BMP581_Interrupt_GPIO_Port GPIOC
#define BMP581_Interrupt_EXTI_IRQn EXTI3_IRQn
#define CONF_CHECK_Pin GPIO_PIN_4
#define CONF_CHECK_GPIO_Port GPIOA
#define FLASH_CS_Pin GPIO_PIN_4
#define FLASH_CS_GPIO_Port GPIOC
#define DEBUG2_Pin GPIO_PIN_5
#define DEBUG2_GPIO_Port GPIOC
#define DEBUG0_Pin GPIO_PIN_0
#define DEBUG0_GPIO_Port GPIOB
#define DEBUG1_Pin GPIO_PIN_1
#define DEBUG1_GPIO_Port GPIOB
#define MMC5983MA_Interrupt_Pin GPIO_PIN_2
#define MMC5983MA_Interrupt_GPIO_Port GPIOB
#define MMC5983MA_Interrupt_EXTI_IRQn EXTI2_IRQn
#define ICM45686_Interrupt_Pin GPIO_PIN_6
#define ICM45686_Interrupt_GPIO_Port GPIOC
#define ICM45686_Interrupt_EXTI_IRQn EXTI9_5_IRQn
#define MMC5983MA_CS_Pin GPIO_PIN_7
#define MMC5983MA_CS_GPIO_Port GPIOC
#define ICM45686_CS_Pin GPIO_PIN_9
#define ICM45686_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
// void JumpToBootloader(void);

void blink();
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
