/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "fault_recovery.h"
#include "led.h"

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
/* USER CODE BEGIN Variables */

/* Lives in .noinit (see the linker script) so it survives the reset the hook below triggers. A
   debugger can read it by name to see which task overflowed last. */
FaultRecord_t g_fault_record __attribute__((section(".noinit")));

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName) {
  /* Run time stack overflow checking is performed if
  configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
  called if a stack overflow is detected. */
  /* The overflowing task has already corrupted memory past its stack, so don't carry on. Reset to
     recover, unless a debugger is attached or the overflow keeps recurring right after boot. This
     runs inside the context switch on the main stack and never returns. */
  __disable_irq();

  bool debugger_attached = (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U;
  FaultAction_t action = fault_recovery_on_stack_overflow(
      &g_fault_record, (const char *)pcTaskName, HAL_GetTick(), debugger_attached);

  if (action == FAULT_ACTION_RESET) {
    HAL_NVIC_SystemReset();
  }

  led_set_status(STACK_OVERFLOW_FAIL);
  if (debugger_attached) {
    __BKPT(0); /* stop right at the overflow so the debugger shows the call stack */
  }
  for (;;) {
  }
}
/* USER CODE END 4 */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

