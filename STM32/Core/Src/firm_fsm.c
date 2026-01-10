#include "firm_fsm.h"

static FIRMState state = FIRM_BOOT;

FSMResponse fsm_process_request(SystemRequest sysreq, TaskCommand* task_command_queue) {
  switch (sysreq) {
    case SYSREQ_START_LIVE:
      // switching to live mode can only be done from boot mode
      if (state != FIRM_BOOT)
        return FSMRES_INVALID;
      
    case SYSREQ_START_MOCK:
      // switching to mock mode can only be done from live mode
      if (state != FIRM_LIVE)
        return FSMRES_INVALID;

    case SYSREQ_CANCEL:
      // cancelling the current command (a mock) can only be done from mock mode
      if (state != FIRM_MOCK)
        return FSMRES_INVALID;

    default:
      return FSMRES_INVALID;
  }
}