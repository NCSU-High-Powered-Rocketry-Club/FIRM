#include "mode_indicator_task.h"

#include "led.h"

void firm_mode_indicator_task(void *argument) {
  (void)argument;

  TaskCommandOption cmd_status = TASKCMD_RESET;
  LED_Mode_Indicator_Status led_setting = FIRM_MODE_DEFAULT;
  led_set_status(led_setting);

  for (;;) {
    if (xQueueReceive(mode_indicator_command_queue, &cmd_status, pdMS_TO_TICKS(100)) == pdTRUE) {
      switch (cmd_status) {
      case TASKCMD_SETUP:
        led_setting = FIRM_MODE_BOOT;
        break;
      case TASKCMD_LIVE:
        led_setting = FIRM_MODE_LIVE;
        break;
      case TASKCMD_MOCK:
      case TASKCMD_MOCK_SETUP:
        led_setting = FIRM_MODE_MOCK;
        break;
      case TASKCMD_RESET:
      default:
        led_setting = FIRM_MODE_DEFAULT;
        break;
      }
    }

    led_toggle_status(led_setting);
  }
}
