#include "sensor_task.h"

osThreadId_t sensor_task_handle;
const osThreadAttr_t sensorTask_attributes = {
    .name = "sensorTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh7,
};

EventGroupHandle_t sensor_event_group;
EventGroupHandle_t sensor_collected_group;
DataPacket_t latest_data_packet;

void sensor_task(void *arg) {
  (void)arg;

  for (;;) {
    EventBits_t fired = xEventGroupWaitBits(
        sensor_event_group,
        SENSOR_EVENT_BAROMETER_READY | SENSOR_EVENT_IMU_READY | SENSOR_EVENT_MAGNETOMETER_READY |
            SENSOR_EVENT_HIGH_G_READY,
        pdTRUE,  // clear on exit
        pdFALSE, // any bit, not all
        portMAX_DELAY);

    if (fired & SENSOR_EVENT_BAROMETER_READY) {
      sensor_collect_data(ID_BAROMETER, &latest_data_packet);
      (void)xEventGroupSetBits(sensor_collected_group, SENSOR_COLLECTED_BAROMETER_BIT);
    }
    if (fired & SENSOR_EVENT_IMU_READY) {
      sensor_collect_data(ID_IMU, &latest_data_packet);
      (void)xEventGroupSetBits(sensor_collected_group, SENSOR_COLLECTED_IMU_BIT);
    }
    if (fired & SENSOR_EVENT_MAGNETOMETER_READY) {
      sensor_collect_data(ID_MAGNETOMETER, &latest_data_packet);
      (void)xEventGroupSetBits(sensor_collected_group, SENSOR_COLLECTED_MAGNETOMETER_BIT);
    }
    if (fired & SENSOR_EVENT_HIGH_G_READY) {
      sensor_collect_data(ID_HIGH_G_ACCELEROMETER, &latest_data_packet);
      (void)xEventGroupSetBits(sensor_collected_group, SENSOR_COLLECTED_HIGH_G_BIT);
    }
  }
}