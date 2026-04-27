#include "mode_indicator_task.h"

#include "led.h"

void firm_mode_indicator_task(void *argument) {
  (void)argument;

  LED_Mode_Indicator_Status led_setting = FIRM_MODE_DEFAULT;
  led_set_status(led_setting);

  for (;;) {
    led_toggle_status(led_setting);
    vTaskDelay(100);
  }
}
