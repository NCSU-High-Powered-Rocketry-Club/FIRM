#include "firm_fsm.h"

static FIRMState state = FIRM_BOOT;

FSMResponse fsm_process_request(SystemRequest sysreq, TaskCommand *task_command_queue) {
  switch (sysreq) {
  case SYSREQ_SETUP:
    if (state != FIRM_BOOT)
      return FSMRES_INVALID;
    task_command_queue[0].target_task = TASK_MODE_INDICATOR;
    task_command_queue[0].command = TASKCMD_SETUP;
    task_command_queue[1].target_task = TASK_DATA_FILTER;
    task_command_queue[1].command = TASKCMD_SETUP;
    task_command_queue[2].target_task = TASK_NULL;
    state = FIRM_SETUP;
    return FSMRES_VALID;

  case SYSREQ_FINISH_SETUP:
    // switching to live mode can only be done from boot mode
    if (state != FIRM_SETUP)
      return FSMRES_INVALID;
    task_command_queue[0].target_task = TASK_MODE_INDICATOR;
    task_command_queue[0].command = TASKCMD_LIVE;
    task_command_queue[1].target_task = TASK_DATA_FILTER;
    task_command_queue[1].command = TASKCMD_LIVE;
    task_command_queue[2].target_task = TASK_NULL;
    state = FIRM_LIVE;
    return FSMRES_VALID;

  case SYSREQ_START_MOCK:
    // switching to mock mode can only be done from live mode
    if (state != FIRM_LIVE) {
      task_command_queue[0].command = TASKCMD_SYSTEM_PACKET_FAILURE;
      return FSMRES_INVALID;
    }
    task_command_queue[0].target_task = TASK_MODE_INDICATOR;
    task_command_queue[0].command = TASKCMD_MOCK;
    task_command_queue[1].target_task = TASK_BMP581;
    task_command_queue[1].command = TASKCMD_MOCK;
    task_command_queue[2].target_task = TASK_ICM45686;
    task_command_queue[2].command = TASKCMD_MOCK;
    task_command_queue[3].target_task = TASK_MMC5983MA;
    task_command_queue[3].command = TASKCMD_MOCK;
    task_command_queue[4].target_task = TASK_DATA_FILTER;
    task_command_queue[4].command = TASKCMD_MOCK_SETUP;
    task_command_queue[5].target_task = TASK_MOCK_PACKET_HANDLER;
    task_command_queue[5].command = TASKCMD_START;
    task_command_queue[6].target_task = TASK_PACKETIZER;
    task_command_queue[6].command = TASKCMD_SYSTEM_PACKET_SUCCESS;
    task_command_queue[7].target_task = TASK_NULL;
    state = FIRM_MOCK_SETUP;
    return FSMRES_VALID;

  case SYSREQ_FINISH_MOCK_SETUP:
    // switching into mock run mode can only be done from mock setup
    if (state != FIRM_MOCK_SETUP)
      return FSMRES_INVALID;
    task_command_queue[0].target_task = TASK_MODE_INDICATOR;
    task_command_queue[0].command = TASKCMD_MOCK;
    task_command_queue[1].target_task = TASK_DATA_FILTER;
    task_command_queue[1].command = TASKCMD_MOCK;
    task_command_queue[2].target_task = TASK_NULL;
    state = FIRM_MOCK;
    return FSMRES_VALID;

  case SYSREQ_CANCEL:
    // cancelling the current command (a mock) can only be done from mock mode
    if (state != FIRM_MOCK && state != FIRM_MOCK_SETUP) {
      // tell packetizer to send invalid packet
      task_command_queue[0].command = TASKCMD_SYSTEM_PACKET_FAILURE;
      return FSMRES_INVALID;
    }
    task_command_queue[0].target_task = TASK_MODE_INDICATOR;
    task_command_queue[0].command = TASKCMD_SETUP;
    task_command_queue[1].target_task = TASK_BMP581;
    task_command_queue[1].command = TASKCMD_LIVE;
    task_command_queue[2].target_task = TASK_ICM45686;
    task_command_queue[2].command = TASKCMD_LIVE;
    task_command_queue[3].target_task = TASK_MMC5983MA;
    task_command_queue[3].command = TASKCMD_LIVE;
    task_command_queue[4].target_task = TASK_DATA_FILTER;
    task_command_queue[4].command = TASKCMD_SETUP;
    task_command_queue[5].target_task = TASK_MOCK_PACKET_HANDLER;
    task_command_queue[5].command = TASKCMD_START;
    task_command_queue[6].target_task = TASK_PACKETIZER;
    task_command_queue[6].command = TASKCMD_SYSTEM_PACKET_SUCCESS;
    task_command_queue[7].target_task = TASK_NULL;
    state = FIRM_SETUP;
    return FSMRES_VALID;
  default:
    return FSMRES_INVALID;
  }
}