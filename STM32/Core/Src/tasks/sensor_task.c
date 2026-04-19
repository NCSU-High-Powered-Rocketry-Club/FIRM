#include "sensor_task.h"

void sensor_task(void *arg) {
  for (;;) {
    EventBits_t fired = xEventGroupWaitBits(sensor_event_group,
                                            EVT_BAROMETER_READY | EVT_IMU_READY |
                                                EVT_MAGNETOMETER_READY | EVT_HIGH_G_READY,
                                            pdTRUE,  // clear on exit
                                            pdFALSE, // any bit, not all
                                            portMAX_DELAY);
    if (fired & EVT_BAROMETER_READY)
      sensor_collect_data(BAROMETER);
    if (fired & EVT_IMU_READY)
      sensor_collect_data(IMU);
    if (fired & EVT_MAGNETOMETER_READY)
      sensor_collect_data(MAGNETOMETER);
    if (fired & EVT_HIGH_G_READY)
      sensor_collect_data(HIGH_G_ACCELEROMETER);
  }
}