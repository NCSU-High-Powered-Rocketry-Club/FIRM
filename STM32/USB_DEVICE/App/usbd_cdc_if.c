/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
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
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "firm_tasks.h"

#include <string.h>
#include <stdint.h>

#include "usbd_def.h"   // USBD_STATE_CONFIGURED
/* USER CODE END INCLUDE */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PRIVATE_DEFINES */

// After we see host interaction (control-line request or OUT data),
// we will wait up to this long for DTR before allowing TX anyway.
// Keeps Linux safe (no early TX before open), but tolerates Windows apps
// that don't assert DTR. This should be fixed later, maybe in FIRM-client?
#ifndef CDC_DTR_GRACE_MS
#define CDC_DTR_GRACE_MS 500U
#endif

/* USER CODE END PRIVATE_DEFINES */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// Control line state (wValue): bit0=DTR, bit1=RTS
volatile uint16_t g_control_line_state = 0;

// Did we ever receive CDC_SET_CONTROL_LINE_STATE?
static volatile uint8_t g_seen_control_line_state = 0;

// Have we received any OUT data or control line requests at all?
static volatile uint8_t g_seen_out_data = 0;

// When host interaction was first observed (ms tick)
static uint32_t g_host_interaction_start_ms = 0;

// TX lock (released on transmit complete)
static volatile uint8_t g_tx_lock = 0;

/* USER CODE END PV */

/* Create buffer for reception and transmission */
/** Received data over USB are stored in this buffer */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

extern USBD_HandleTypeDef hUsbDeviceFS;

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
static uint8_t cdc_tx_allowed_now(void);
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);

  g_control_line_state = 0;
  g_seen_control_line_state = 0;
  g_seen_out_data = 0;
  g_host_interaction_start_ms = 0;
  g_tx_lock = 0;

  return (USBD_OK);
  /* USER CODE END 3 */
}

static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  (void)length;

  switch (cmd)
  {
    // This is where DTR arrives from Linux
    case CDC_SET_CONTROL_LINE_STATE:
      g_control_line_state = (uint16_t)(pbuf[0] | ((uint16_t)pbuf[1] << 8));
      g_seen_control_line_state = 1U;

      if (g_host_interaction_start_ms == 0U) {
        g_host_interaction_start_ms = HAL_GetTick();
      }
      break;

    case CDC_SET_LINE_CODING:
    case CDC_GET_LINE_CODING:
    case CDC_SEND_ENCAPSULATED_COMMAND:
    case CDC_GET_ENCAPSULATED_RESPONSE:
    case CDC_SET_COMM_FEATURE:
    case CDC_GET_COMM_FEATURE:
    case CDC_CLEAR_COMM_FEATURE:
    case CDC_SEND_BREAK:
    default:
      break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  g_seen_out_data = 1U;
  if (g_host_interaction_start_ms == 0U) {
    g_host_interaction_start_ms = HAL_GetTick();
  }

  usb_receive_callback(Buf, *Len);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

static uint8_t cdc_tx_allowed_now(void)
{
  // Must be enumerated
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
    return 0U;
  }

  // Best case: host asserts DTR -> allowed
  if ((g_control_line_state & 0x0001U) != 0U) {
    return 1U;
  }

  // CRITICAL for Linux:
  // Do NOT allow TX before we've seen any sign the host/app actually opened/talked
  // to the CDC interface. (Otherwise you can "hang" the first TX.)
  if ((g_seen_out_data == 0U) && (g_seen_control_line_state == 0U)) {
    return 0U;
  }

  // Host is interacting but DTR isn't asserted (common on some Windows apps).
  // After a short grace window, allow TX anyway.
  if (g_host_interaction_start_ms == 0U) {
    g_host_interaction_start_ms = HAL_GetTick();
    return 0U;
  }

  if ((HAL_GetTick() - g_host_interaction_start_ms) >= CDC_DTR_GRACE_MS) {
    return 1U;
  }

  return 0U;
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */

  if (!cdc_tx_allowed_now()) {
    return USBD_BUSY;
  }

  // Prevent multi-task collision / reentrancy
  if (__atomic_test_and_set(&g_tx_lock, __ATOMIC_ACQUIRE)) {
    return USBD_BUSY;
  }

  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if ((hcdc == NULL) || (hcdc->TxState != 0U)) {
    __atomic_clear(&g_tx_lock, __ATOMIC_RELEASE);
    return USBD_BUSY;
  }

  if (Len > (uint16_t)APP_TX_DATA_SIZE) {
    __atomic_clear(&g_tx_lock, __ATOMIC_RELEASE);
    return USBD_FAIL;
  }

  // Copy into stable buffer (caller Buf may be transient)
  if (Len > 0U) {
    (void)memcpy(UserTxBufferFS, Buf, Len);
  }

  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);

  // If we couldn't start, release immediately
  if (result != USBD_OK) {
    __atomic_clear(&g_tx_lock, __ATOMIC_RELEASE);
  }

  /* USER CODE END 7 */
  return result;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);

  // Release TX lock on completion
  __atomic_clear(&g_tx_lock, __ATOMIC_RELEASE);

  /* USER CODE END 13 */
  return result;
}
