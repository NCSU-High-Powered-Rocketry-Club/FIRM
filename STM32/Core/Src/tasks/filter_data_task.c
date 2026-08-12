#include "filter_data_task.h"

#include "sensor_task.h"
#include "system_state.h"

#include "FreeRTOS.h"
#include "error_state_kalman_filter.h"
#include "eskf_functions.h"
#include "task.h"

osThreadId_t filter_data_task_handle;
const osThreadAttr_t filterDataTask_attributes = {
    .name = "filterDataTask",
    .stack_size = 4096 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

static bool has_initial_sensor_data(const DataPacket_t *data) {
  return data->magnetic_field_x_microteslas != 0.0F && data->pressure_pascals != 0.0F;
}

void filter_data_task(void *arg) {
  DataPacket_t *data_packet = (DataPacket_t *)arg;

  ESKF eskf;
  DataPacket_t snapshot;

  for (;;) {
    snapshot = *data_packet;
    if (has_initial_sensor_data(&snapshot)) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  TickType_t start_time = xTaskGetTickCount();
  vTaskDelay(pdMS_TO_TICKS(100));

  while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(KALMAN_FILTER_STARTUP_DELAY_TIME_MS)) {
    vTaskDelay(pdMS_TO_TICKS(5));

    snapshot = *data_packet;

    eskf_accumulate(snapshot.pressure_pascals, &snapshot.raw_acceleration_x_gs,
                    &snapshot.magnetic_field_x_microteslas);
  }

  (void)eskf_init(&eskf);
  system_state_set(SYSTEM_STATE_LIVE);

  float last_time = (float)snapshot.timestamp_seconds - 0.005F;

  for (;;) {
    (void)xEventGroupWaitBits(sensor_collected_group,
                              SENSOR_COLLECTED_BAROMETER_BIT | SENSOR_COLLECTED_IMU_BIT |
                                  SENSOR_COLLECTED_MAGNETOMETER_BIT,
                              pdTRUE, pdTRUE, portMAX_DELAY);

    snapshot = *data_packet;

    ESKFRawData raw_data = {0};
    raw_data.timestamp_seconds = snapshot.timestamp_seconds;
    raw_data.pressure_pascals = snapshot.pressure_pascals;
    raw_data.raw_acceleration_x_gs = snapshot.raw_acceleration_x_gs;
    raw_data.raw_acceleration_y_gs = snapshot.raw_acceleration_y_gs;
    raw_data.raw_acceleration_z_gs = snapshot.raw_acceleration_z_gs;
    raw_data.raw_angular_rate_x_deg_per_s = snapshot.raw_angular_rate_x_dps;
    raw_data.raw_angular_rate_y_deg_per_s = snapshot.raw_angular_rate_y_dps;
    raw_data.raw_angular_rate_z_deg_per_s = snapshot.raw_angular_rate_z_dps;
    raw_data.magnetic_field_x_microteslas = snapshot.magnetic_field_x_microteslas;
    raw_data.magnetic_field_y_microteslas = snapshot.magnetic_field_y_microteslas;
    raw_data.magnetic_field_z_microteslas = snapshot.magnetic_field_z_microteslas;

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

    // memcpy(&data_packet->est_position_z_meters, eskf.x_nom, ESKF_NOMINAL_DIM * sizeof(float));
  }
}
