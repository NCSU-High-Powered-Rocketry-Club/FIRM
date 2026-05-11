#include "mode_indicator_task.h"

#include "led.h"
#include "system_state.h"

#include "FreeRTOS.h"
#include "task.h"

osThreadId_t firm_mode_indicator_task_handle;
const osThreadAttr_t modeIndicatorTask_attributes = {
    .name = "modeIndicatorTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

static LED_Mode_Indicator_Status led_status_from_state(SystemState_t state) {
  switch (state) {
  case SYSTEM_STATE_BOOT:
    return FIRM_MODE_BOOT;
  case SYSTEM_STATE_MOCK:
    return FIRM_MODE_MOCK;
  case SYSTEM_STATE_LIVE:
  default:
    return FIRM_MODE_LIVE;
  }
}

void firm_mode_indicator_task(void *argument) {
  (void)argument;

  LED_Mode_Indicator_Status led_setting = led_status_from_state(system_state_get());
  led_set_status(led_setting);

  for (;;) {
    LED_Mode_Indicator_Status desired = led_status_from_state(system_state_get());
    if (desired != led_setting) {
      led_setting = desired;
      led_set_status(led_setting);
    }
    led_toggle_status(led_setting);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
