#include "sensor_task.h"

osThreadId_t sensor_task_handle;
const osThreadAttr_t sensorTask_attributes = {
    .name = "sensorTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh7,
};

EventGroupHandle_t sensor_event_group;
EventGroupHandle_t sensor_collected_group;

void sensor_task(void *arg) {
  DataPacket_t *data_packet = (DataPacket_t *)arg;

  for (;;) {
    EventBits_t fired = xEventGroupWaitBits(
        sensor_event_group,
        SENSOR_EVENT_BAROMETER_READY | SENSOR_EVENT_IMU_READY | SENSOR_EVENT_MAGNETOMETER_READY |
            SENSOR_EVENT_HIGH_G_READY,
        pdTRUE,  // clear on exit
        pdFALSE, // any bit, not all
        portMAX_DELAY);

    if (fired & SENSOR_EVENT_BAROMETER_READY) {
      sensor_collect_data(ID_BAROMETER, data_packet);
      (void)xEventGroupSetBits(sensor_collected_group, SENSOR_COLLECTED_BAROMETER_BIT);
    }
    if (fired & SENSOR_EVENT_IMU_READY) {
      sensor_collect_data(ID_IMU, data_packet);
      (void)xEventGroupSetBits(sensor_collected_group, SENSOR_COLLECTED_IMU_BIT);
    }
    if (fired & SENSOR_EVENT_MAGNETOMETER_READY) {
      sensor_collect_data(ID_MAGNETOMETER, data_packet);
      (void)xEventGroupSetBits(sensor_collected_group, SENSOR_COLLECTED_MAGNETOMETER_BIT);
    }
    if (fired & SENSOR_EVENT_HIGH_G_READY) {
      sensor_collect_data(ID_HIGH_G_ACCELEROMETER, data_packet);
      (void)xEventGroupSetBits(sensor_collected_group, SENSOR_COLLECTED_HIGH_G_BIT);
    }
  }
}