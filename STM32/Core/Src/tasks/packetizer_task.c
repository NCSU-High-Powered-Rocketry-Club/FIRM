#include "packetizer_task.h"

#include "messages.h"
#include "settings_manager.h"
#include <string.h>

void packetizer_task(void *argument) {
  (void)argument;

  const SystemSettings_t *settings = get_settings();
  uint16_t frequency_hz = (settings->frequency_hz == 0U) ? 1U : settings->frequency_hz;
  const TickType_t transmit_freq = MAX_WAIT_TIME(frequency_hz);
  TickType_t last_wake_time = xTaskGetTickCount();

  for (;;) {
    TransmitFrame_t frame = {0};
    frame.payload[0] = (uint8_t)ID_DATA_PACKET;

    osMutexAcquire(sensorDataMutexHandle, osWaitForever);
    memcpy(&frame.payload[1], &latest_data_packet, sizeof(latest_data_packet));
    osMutexRelease(sensorDataMutexHandle);

    frame.payload_len = (uint16_t)(1U + sizeof(DataPacket_t));
    if (frame.payload_len <= MAX_TRANSMIT_PAYLOAD_LEN) {
      (void)xQueueSend(transmit_queue, &frame, 0);
    }

    vTaskDelayUntil(&last_wake_time, transmit_freq);
  }
}
