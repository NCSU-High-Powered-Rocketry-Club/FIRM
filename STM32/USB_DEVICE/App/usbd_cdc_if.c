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
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* READ THIS:
    Windows/Linux interact with USB slightly differently. When the host opens the 
    COM port, Windows will assert DTR immediately, while Linux usually waits to
    read/write until DTR is asserted. This means that on Windows, you can just
    connect to the COM port and start sending data, while on Linux, you need to  
    wait for the COM port to be opened before sending data. 
    
    To make this work on both platforms, firm-client will assert DTR before
    sending data.
 */

/** Stores info about the usb connection. */
static USBD_CDC_LineCodingTypeDef g_line_coding = {
  115200U,
  0U,
  0U,
  8U,
};

/** Stores the last DTR/RTS bitfield from the host */
static volatile uint16_t g_control_line_state = 0U;
/** This is set to 1 when the port is open (DTR asserted) */
static volatile uint8_t g_port_open = 0U;
/** If this is 1, something is currently transmitting */
static volatile uint8_t g_tx_lock = 0U;

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Sets the Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);

  // Tells the CDC driver where to place received data
  (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);

  // Arms the endpoint, basically tells the host that the device is ready to receive data.
  g_control_line_state = 0U; // resets DTR/RTS tracking
  g_port_open = 0U; // resets the open flag
  // Reset the TX lock
  __atomic_clear(&g_tx_lock, __ATOMIC_RELEASE);

  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  // Just handles the USB disconnecting
  g_control_line_state = 0U;
  g_port_open = 0U;
  __atomic_clear(&g_tx_lock, __ATOMIC_RELEASE);
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    // Handles setting the serial settings that the host wants, but these settings aren't actually 
    // used for anything since this is a virtual com port and not a real UART
    case CDC_SET_LINE_CODING:
      if (length >= 7U)
      {
        g_line_coding.bitrate = (uint32_t)pbuf[0]
                             | ((uint32_t)pbuf[1] << 8)
                             | ((uint32_t)pbuf[2] << 16)
                             | ((uint32_t)pbuf[3] << 24);
        g_line_coding.format = pbuf[4];
        g_line_coding.paritytype = pbuf[5];
        g_line_coding.datatype = pbuf[6];
      }
    break;
    // Tells the host the current serial settings, but again these aren't actually used for anything
    case CDC_GET_LINE_CODING:
      pbuf[0] = (uint8_t)(g_line_coding.bitrate);
      pbuf[1] = (uint8_t)(g_line_coding.bitrate >> 8);
      pbuf[2] = (uint8_t)(g_line_coding.bitrate >> 16);
      pbuf[3] = (uint8_t)(g_line_coding.bitrate >> 24);
      pbuf[4] = g_line_coding.format;
      pbuf[5] = g_line_coding.paritytype;
      pbuf[6] = g_line_coding.datatype;
    break;

    // Handles the host opening and closing the COM port by tracking the DTR bit
    case CDC_SET_CONTROL_LINE_STATE:
    {
      /* wValue bit0 = DTR, bit1 = RTS */
      g_control_line_state = hUsbDeviceFS.request.wValue;
      // Update the port open flag based on the DTR bit
      g_port_open = ((g_control_line_state & 0x0001U) != 0U) ? 1U : 0U;
    }
    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  // If the port isn't open, just drop the received data and return
  if (!g_port_open)
  {
    // Says to put the next packet the host sends into the same buffer
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    // Re-Arms the endpoint to receive the next packet
    (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return (USBD_OK);
  }

  // This is the point we give data to firm_tasks
  usb_receive_callback(Buf, *Len);

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  // If the port isn't configured or open yet, don't send anything
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || !g_port_open)
  {
    return USBD_BUSY;
  }

  // Check for buffer overflow
  if (Len > (uint16_t)APP_TX_DATA_SIZE)
  {
    return USBD_FAIL;
  }

  // Check if we can take the TX lock, if we can then lock it and proceed with transmission
  if (__atomic_test_and_set(&g_tx_lock, __ATOMIC_ACQUIRE))
  {
    return USBD_BUSY;
  }

  // Get the CDC handle
  USBD_CDC_HandleTypeDef *hcdc =
    (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassDataCmsit[hUsbDeviceFS.classId];

  // If the CDC handle isn't valid or a transmission is already in progress, release the lock and return busy
  if ((hcdc == NULL) || (hcdc->TxState != 0U))
  {
    __atomic_clear(&g_tx_lock, __ATOMIC_RELEASE);
    return USBD_BUSY;
  }

  if (Len > 0U)
  {
    (void)memcpy(UserTxBufferFS, Buf, Len);
  }

  // Set the buffer and length, then start the transmission
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);

  if (result != USBD_OK)
  {
    __atomic_clear(&g_tx_lock, __ATOMIC_RELEASE);
  }
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  __atomic_clear(&g_tx_lock, __ATOMIC_RELEASE);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
