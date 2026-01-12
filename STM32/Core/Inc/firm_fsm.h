#pragma once

#define MAX_TASK_COMMANDS 15

typedef enum {
  FIRM_BOOT,
  FIRM_SETUP,
  FIRM_LIVE,
  FIRM_MOCK,
} FIRMState;

typedef enum {
  SYSREQ_SETUP,
  SYSREQ_FINISH_SETUP,
  SYSREQ_START_MOCK,
  SYSREQ_CANCEL,
} SystemRequest;

typedef enum {
  TASKCMD_RESET,
  TASKCMD_MOCK,
  TASKCMD_LIVE,
  TASKCMD_SETUP,
} TaskCommandOption;

typedef enum {
  TASK_BMP581,
  TASK_ICM45686,
  TASK_MMC5983MA,
  TASK_DATA_FILTER,
  TASK_USB_TRANSMIT,
  TASK_UART_TRANSMIT,
  TASK_MODE_INDICATOR,
  TASK_NULL,
} FIRMTask;

typedef struct {
  FIRMTask target_task;
  TaskCommandOption command;
} TaskCommand;

typedef enum {
  FSMRES_VALID,
  FSMRES_INVALID,
} FSMResponse;

FSMResponse fsm_process_request(SystemRequest sysreq, TaskCommand* task_command_queue);