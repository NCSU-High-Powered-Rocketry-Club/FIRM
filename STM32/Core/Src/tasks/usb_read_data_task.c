#include "usb_read_data_task.h"

void usb_read_data(void *argument) {
  uint8_t received_bytes[MESSAGE_BYTE_BUFFER_SIZE];
  UsbMessageMeta meta;

  for (;;) {
    xStreamBufferReceive(usb_rx_stream, meta_bytes, sizeof(meta_bytes), portMAX_DELAY);
    if (!usb_parse_message_meta(meta_bytes, sizeof(meta_bytes), &meta)) {
      continue;
    }

    UsbMessageType type = usb_interpret_usb_message(&meta);
    if (type == USBMSG_MOCK_SENSOR) {
      static uint32_t mock_accumulated_clock_cycles = 0U;

      size_t got =
          xStreamBufferReceive(usb_rx_stream, received_bytes, meta.payload_length, portMAX_DELAY);
      if (got != meta.payload_length) {
        // Drop CRC bytes for malformed payload and continue.
        xStreamBufferReceive(usb_rx_stream, received_bytes, 2, portMAX_DELAY);
        continue;
      }

      if (!mock_ring_push_sensor_instance((MockPacketID)meta.identifier, received_bytes,
                                          meta.payload_length)) {
        xStreamBufferReceive(usb_rx_stream, received_bytes, 2, portMAX_DELAY);
        continue;
      }

      // mock payload format begins with 4-byte cycle timestamp.
      uint32_t delay_ms =
          mock_timestamp_accumulate_delay_ms(&mock_accumulated_clock_cycles, received_bytes);
      if (delay_ms > 0U) {
        vTaskDelay(pdMS_TO_TICKS(1));
      }

      // Inject mock adapters so sensor_manager reads from mock ring instead of live drivers.
      set_time_fn(mock_adapter_get_time);
      set_barometer_read_fn(mock_adapter_read_bmp581);
      set_imu_read_fn(mock_adapter_read_icm45686);
      set_magnetometer_read_fn(mock_adapter_read_mmc5983ma);
      set_high_g_read_fn(mock_adapter_read_adxl371);
      mock_adapter_reset_cached_instance();

      switch ((MockPacketID)meta.identifier) {
      case MOCKID_BMP581:
        sensor_collect_data(BAROMETER, &data_packet.data.data_packet);
        break;
      case MOCKID_ICM45686:
        sensor_collect_data(IMU, &data_packet.data.data_packet);
        break;
      case MOCKID_MMC5983MA:
        sensor_collect_data(MAGNETOMETER, &data_packet.data.data_packet);
        break;
      case MOCKID_ADXL371:
        sensor_collect_data(HIGH_G_ACCELEROMETER, &data_packet.data.data_packet);
        break;
      default:
        break;
      }

      xStreamBufferReceive(usb_rx_stream, received_bytes, 2, portMAX_DELAY); // discard CRC bytes
      continue;
    }

    // read bytes and validate crc
    while (xStreamBufferBytesAvailable(usb_rx_stream) < meta.payload_length + 2) {
      __NOP();
    }
    xStreamBufferReceive(usb_rx_stream, received_bytes, meta.payload_length + 2, portMAX_DELAY);
    if (!validate_message_crc16(meta.header, meta.identifier, meta.payload_length,
                                received_bytes)) {
      continue; // invalid crc, skip
    }

    switch (type) {
    case USBMSG_MOCK_SETTINGS: {
      SystemSettings_t mock_settings = {0};
      SensorScaleFactors_t scale_factors;
      if (process_mock_settings_packet(received_bytes, meta.payload_length, &mock_settings,
                                       &scale_factors)) {
        logger_append_mock_header(&mock_settings, &scale_factors);
      }
      if (mock_ring_mutex != NULL) {
        (void)xSemaphoreTake(mock_ring_mutex, portMAX_DELAY);
      }
      mock_ring_reset(&mock_ring); // reset the ring buffer to prepare for packets
      if (mock_ring_mutex != NULL) {
        (void)xSemaphoreGive(mock_ring_mutex);
      }
      mock_timestamp_accumulator_reset();
      break;
    }
    case USBMSG_COMMAND: {
      Packet response;
      response.packet_len = commands_execute_to_response(
          meta.identifier, received_bytes, meta.payload_length, &response.header,
          &response.identifier, (ResponsePacket *)&response.data);
      xQueueSend(transmit_queue, &response, portMAX_DELAY);
      break;
    }
    case USBMSG_SYSTEM_REQUEST:
      xQueueSend(system_request_queue, (SystemRequest *)&meta.identifier, portMAX_DELAY);
      break;
    // mock sensor should be processed already
    case USBMSG_MOCK_SENSOR:
    case USBMSG_INVALID:
    default:
      break;
    }
  }
}

void usb_receive_callback(const uint8_t *buffer, uint32_t data_length) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Check if the stream buffer is initialized
  if (usb_rx_stream != NULL) {
    // Adds the received data to the stream buffer from an ISR context
    xStreamBufferSendFromISR(usb_rx_stream, buffer, data_length, &xHigherPriorityTaskWoken);
  }
  // If the usb_read_data task has a higher priority than the currently running task, request a
  // context switch
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
