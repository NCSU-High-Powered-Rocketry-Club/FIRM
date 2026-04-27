#include "mock_packet_handler_task.h"

#include "sensor_task.h"

#include "data_processing/mocking_ring_buffer.h"

static TaskHandle_t task_for_mock_identifier(Identifiers_t id) {
  switch (id) {
  case ID_BAROMETER:
    return bmp581_task_handle;
  case ID_IMU:
    return icm45686_task_handle;
  case ID_MAGNETOMETER:
    return mmc5983ma_task_handle;
  case ID_HIGH_G_ACCELEROMETER:
    return adxl371_task_handle;
  default:
    return NULL;
  }
}

void mock_packet_handler(void *argument) {
  (void)argument;

  TaskCommandOption cmd_status = TASKCMD_RESET;

  for (;;) {
    (void)xQueueReceive(mock_packet_handler_command_queue, &cmd_status, 0);

    if (cmd_status != TASKCMD_START) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    const uint8_t *peek = (const uint8_t *)mock_ring_peek();
    if (peek == NULL) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    TaskHandle_t sensor_task = task_for_mock_identifier((Identifiers_t)peek[0]);
    if (sensor_task == NULL) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    (void)xTaskNotify(sensor_task, SENSOR_NOTIFY_MOCK_BIT, eSetBits);
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
  }
}
