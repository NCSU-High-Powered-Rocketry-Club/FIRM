#include "filter_data_task.h"

#include "sensor_task.h"

#include "error_state_kalman_filter.h"
#include "eskf_functions.h"
#include <string.h>

void filter_data_task(void *argument) {
  (void)argument;

  ESKF eskf;
  memset(&eskf, 0, sizeof(ESKF));
  float last_time = 0.0F;
  bool setup = true;

  for (;;) {

    if (setup) {
      DataPacket_t snapshot = {0};
      snapshot = latest_data_packet;

      if (snapshot.magnetic_field_x_microteslas == 0.0F || snapshot.pressure_pascals == 0.0F) {
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }

      TickType_t start_time = xTaskGetTickCount();
      vTaskDelay(pdMS_TO_TICKS(100));

      while ((xTaskGetTickCount() - start_time) <
             pdMS_TO_TICKS(KALMAN_FILTER_STARTUP_DELAY_TIME_MS)) {
        vTaskDelay(pdMS_TO_TICKS(5));

        snapshot = latest_data_packet;

        eskf_accumulate(snapshot.pressure_pascals, &snapshot.raw_acceleration_x_gs,
                        &snapshot.magnetic_field_x_microteslas);
      }

      setup = false;

      (void)eskf_init(&eskf);

      last_time = (float)latest_data_packet.timestamp_seconds - 0.005F;
    }

    ESKFRawData raw_data = {0};
    raw_data.timestamp_seconds = latest_data_packet.timestamp_seconds;
    raw_data.pressure_pascals = latest_data_packet.pressure_pascals;
    raw_data.raw_acceleration_x_gs = latest_data_packet.raw_acceleration_x_gs;
    raw_data.raw_acceleration_y_gs = latest_data_packet.raw_acceleration_y_gs;
    raw_data.raw_acceleration_z_gs = latest_data_packet.raw_acceleration_z_gs;
    raw_data.raw_angular_rate_x_deg_per_s = latest_data_packet.raw_angular_rate_x_dps;
    raw_data.raw_angular_rate_y_deg_per_s = latest_data_packet.raw_angular_rate_y_dps;
    raw_data.raw_angular_rate_z_deg_per_s = latest_data_packet.raw_angular_rate_z_dps;
    raw_data.magnetic_field_x_microteslas = latest_data_packet.magnetic_field_x_microteslas;
    raw_data.magnetic_field_y_microteslas = latest_data_packet.magnetic_field_y_microteslas;
    raw_data.magnetic_field_z_microteslas = latest_data_packet.magnetic_field_z_microteslas;

    float dt = (float)raw_data.timestamp_seconds - last_time;
    if (dt <= 1e-6F) {
      continue;
    }

    last_time = (float)raw_data.timestamp_seconds;

    float u[ESKF_CONTROL_DIM] = {
        raw_data.raw_acceleration_x_gs,        raw_data.raw_acceleration_y_gs,
        raw_data.raw_acceleration_z_gs,        raw_data.raw_angular_rate_x_deg_per_s,
        raw_data.raw_angular_rate_y_deg_per_s, raw_data.raw_angular_rate_z_deg_per_s,
    };

    eskf_predict(&eskf, u, dt);

    float z_raw[ESKF_MEASUREMENT_DIM] = {
        raw_data.pressure_pascals,
        raw_data.magnetic_field_x_microteslas,
        raw_data.magnetic_field_y_microteslas,
        raw_data.magnetic_field_z_microteslas,
    };

    eskf_set_measurement(&eskf, z_raw);
    eskf_update(&eskf);

    memcpy(&latest_data_packet.est_position_z_meters, eskf.x_nom, ESKF_NOMINAL_DIM * sizeof(float));

    (void)xEventGroupWaitBits(sensors_collected,
                              BMP581_TASK_BIT | ICM45686_TASK_BIT | MMC5983MA_TASK_BIT, pdTRUE,
                              pdTRUE, portMAX_DELAY);
  }
}
