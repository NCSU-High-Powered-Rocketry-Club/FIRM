#include "system_manager_task.h"

#include "filter_data_task.h"
#include "mock_packet_handler_task.h"
#include "mode_indicator_task.h"
#include "sensor_task.h"

static bool system_manager_wait_for_step_ack(FIRMTask task, TaskCommandOption command) {
  (void)task;
  (void)command;
  // TODO(system-manager-refactor): wait for explicit per-step task acknowledgements.
  return true;
}

static QueueHandle_t system_manager_command_queue_for_task(FIRMTask task) {
  switch (task) {
  case TASK_BMP581:
    return bmp581_command_queue;
  case TASK_ICM45686:
    return icm45686_command_queue;
  case TASK_MMC5983MA:
    return mmc5983ma_command_queue;
  case TASK_ADXL371:
    return adxl371_command_queue;
  case TASK_DATA_FILTER:
    // TODO(system-manager-refactor): route filter through a refactored endpoint contract.
    return data_filter_command_queue;
  case TASK_MODE_INDICATOR:
    // TODO(system-manager-refactor): route LED/mode indicator through a refactored endpoint contract.
    return mode_indicator_command_queue;
  case TASK_MOCK_PACKET_HANDLER:
    return mock_packet_handler_command_queue;
  case TASK_PACKETIZER:
  case TASK_NULL:
  default:
    return NULL;
  }
}

static bool system_manager_dispatch_step(TaskCommand cmd) {
  if (cmd.target_task == TASK_NULL) {
    return true;
  }

  if (cmd.target_task == TASK_PACKETIZER) {
    // Responses are now emitted by commands/USB flow instead of the system manager path.
    return true;
  }

  QueueHandle_t target_queue = system_manager_command_queue_for_task(cmd.target_task);
  if (target_queue == NULL) {
    return false;
  }

  if (xQueueSend(target_queue, &cmd.command, portMAX_DELAY) != pdTRUE) {
    return false;
  }

  return system_manager_wait_for_step_ack(cmd.target_task, cmd.command);
}

static bool system_manager_process_request(SystemRequest request) {
  TaskCommand task_command_queue[MAX_TASK_COMMANDS];
  FSMResponse response = fsm_process_request(request, task_command_queue);

  if (response != FSMRES_VALID) {
    return false;
  }

  for (int i = 0; i < MAX_TASK_COMMANDS; i++) {
    TaskCommand cmd = task_command_queue[i];
    if (cmd.target_task == TASK_NULL) {
      break;
    }

    if (!system_manager_dispatch_step(cmd)) {
      return false;
    }
  }

  return true;
}

static bool system_request_to_identifier(SystemRequest request, Identifiers_t *id_out) {
  if (id_out == NULL) {
    return false;
  }

  switch (request) {
  case SYSREQ_START_MOCK:
    *id_out = ID_MOCK_REQUEST;
    return true;
  case SYSREQ_CANCEL:
    *id_out = ID_CANCEL_REQUEST;
    return true;
  default:
    return false;
  }
}

bool system_manager_submit_request(SystemRequest request, TickType_t timeout_ticks) {
  if (system_request_queue == NULL) {
    return false;
  }

  Identifiers_t id = ID_DATA_PACKET;
  if (system_request_to_identifier(request, &id) && !sys_manager_check_command_valid(id)) {
    return false;
  }

  return xQueueSend(system_request_queue, &request, timeout_ticks) == pdTRUE;
}

void system_manager_task(void *argument) {
  (void)argument;

  SystemRequest request;
  for (;;) {
    if (xQueueReceive(system_request_queue, &request, portMAX_DELAY) == pdTRUE) {
      (void)system_manager_process_request(request);
    }
  }
}
