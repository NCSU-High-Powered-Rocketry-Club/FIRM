#pragma once
#include <stdbool.h>
#include <stdint.h>

#define MAX_TASK_COMMANDS 15

typedef enum {
  FIRM_BOOT,
  FIRM_SETUP,
  FIRM_LIVE,
  FIRM_MOCK_SETUP,
  FIRM_MOCK,
} FIRMState;

typedef enum {
  SYSREQ_SETUP = 0,
  SYSREQ_FINISH_SETUP = 1,
  SYSREQ_START_MOCK = 0x0005, // command sent by host
  SYSREQ_FINISH_MOCK_SETUP = 0x0006, // internal transition
  SYSREQ_CANCEL = 0x00FF, // command sent by host
} SystemRequest;

typedef enum {
  TASKCMD_RESET,
  TASKCMD_MOCK,
  TASKCMD_MOCK_SETUP,
  TASKCMD_LIVE,
  TASKCMD_SETUP,
  TASKCMD_START,
  TASKCMD_SYSTEM_PACKET_SUCCESS,
  TASKCMD_SYSTEM_PACKET_FAILURE,
} TaskCommandOption;

typedef enum {
  TASK_BMP581,
  TASK_ICM45686,
  TASK_MMC5983MA,
  TASK_ADXL371,
  TASK_DATA_FILTER,
  TASK_MODE_INDICATOR,
  TASK_PACKETIZER,
  TASK_MOCK_PACKET_HANDLER,
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

typedef struct {
  uint16_t identifier;
  bool success;
} SystemResponsePacket;

FSMResponse fsm_process_request(SystemRequest sysreq, TaskCommand* task_command_queue);