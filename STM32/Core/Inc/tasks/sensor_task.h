#pragma once

#include "sensor_manager.h"
#include "packets.h"
#include "shared_data/identifiers.h"

#include "cmsis_os.h"
#include "event_groups.h"

#define BMP581_POLL_RATE_HZ 255
#define ICM45686_POLL_RATE_HZ 400
#define MMC5983MA_POLL_RATE_HZ 225
#define ADXL371_POLL_RATE_HZ 160

// Event bits for data-ready interrupts.
#define SENSOR_EVENT_BAROMETER_READY (1U << 0)
#define SENSOR_EVENT_IMU_READY (1U << 1)
#define SENSOR_EVENT_MAGNETOMETER_READY (1U << 2)
#define SENSOR_EVENT_HIGH_G_READY (1U << 3)

// Event bits for sensor data collected.
#define SENSOR_COLLECTED_BAROMETER_BIT (1U << 0)
#define SENSOR_COLLECTED_IMU_BIT (1U << 1)
#define SENSOR_COLLECTED_MAGNETOMETER_BIT (1U << 2)
#define SENSOR_COLLECTED_HIGH_G_BIT (1U << 3)

extern osThreadId_t sensor_task_handle;
extern const osThreadAttr_t sensorTask_attributes;

extern EventGroupHandle_t sensor_event_group;
extern EventGroupHandle_t sensor_collected_group;
extern DataPacket_t latest_data_packet;

void sensor_task(void *argument);