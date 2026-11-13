#include "mode_indicator_task.h"

#include "led.h"

#include "FreeRTOS.h"
#include "task.h"

osThreadId_t firm_mode_indicator_task_handle;
const osThreadAttr_t modeIndicatorTask_attributes = {
    .name = "modeIndicatorTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void firm_mode_indicator_task(void *argument) {
  (void)argument;

  led_set_status(FIRM_MODE_LIVE);

  for (;;) {
    led_toggle_status(FIRM_MODE_LIVE);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
