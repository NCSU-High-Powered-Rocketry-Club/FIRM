#include "firm_tasks.h"

#include "bmp581.h"
#include "icm45686.h"
#include "logger.h"
#include "preprocessor.h"

// task handles
osThreadId_t bmp581_task_handle;
osThreadId_t icm45686_task_handle;
osThreadId_t mmc5983ma_task_handle;

// task attributes
const osThreadAttr_t bmp581Task_attributes = {
    .name = "bmp581Task",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
const osThreadAttr_t icm45686Task_attributes = {
    .name = "icm45686Task",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
const osThreadAttr_t mmc5983maTask_attributes = {
    .name = "mmc5983maTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};

// mutexes
osMutexId_t sensorDataMutexHandle;
const osMutexAttr_t sensorDataMutex_attributes = {
  .name = "sensorDataMutex"
};

void collect_bmp581_data_task(void *argument) {
  const TickType_t max_wait = MAX_WAIT_TIME(BMP581_POLL_RATE_HZ);
  uint32_t notif_count = 0;

  for (;;) {
    notif_count = ulTaskNotifyTake(pdFALSE, max_wait);

    if (notif_count > 0) {
      osMutexAcquire(sensorDataMutexHandle, osWaitForever);
      BMP581Packet_t *bmp581_packet = logger_malloc_packet(sizeof(BMP581Packet_t));
      if (!bmp581_read_data(bmp581_packet)) {
        logger_write_entry('B', sizeof(BMP581Packet_t));
        // bmp581_convert_packet(bmp581_packet, &calibrated_packet);
      }
      osMutexRelease(sensorDataMutexHandle);
    } else {
      // TODO: error handling
    }
  }
}

void collect_icm45686_data_task(void *argument) {
  const TickType_t max_wait = MAX_WAIT_TIME(ICM45686_POLL_RATE_HZ);
  uint32_t notif_count = 0;
  for (;;) {
    notif_count = ulTaskNotifyTake(pdFALSE, max_wait);

    if (notif_count > 0) {
      osMutexAcquire(sensorDataMutexHandle, osWaitForever);
      ICM45686Packet_t *icm45686_packet = logger_malloc_packet(sizeof(ICM45686Packet_t));
      if (!icm45686_read_data(icm45686_packet)) {
        logger_write_entry('I', sizeof(ICM45686Packet_t));
        // icm45686_convert_packet(icm45686_packet, &calibrated_packet);
      }
      osMutexRelease(sensorDataMutexHandle);
    } else {
      // TODO: error handling
    }
  }
}

void collect_mmc5983ma_data_task(void *argument) {
  const TickType_t max_wait = MAX_WAIT_TIME(ICM45686_POLL_RATE_HZ);
  uint32_t notif_count = 0;
  for (;;) {
    notif_count = ulTaskNotifyTake(pdFALSE, max_wait);

    if (notif_count > 0) {
      osMutexAcquire(sensorDataMutexHandle, osWaitForever);
      MMC5983MAPacket_t *mmc5983ma_packet = logger_malloc_packet(sizeof(MMC5983MAPacket_t));
      if (!mmc5983ma_read_data(mmc5983ma_packet, 0)) {
        logger_write_entry('M', sizeof(MMC5983MAPacket_t));
        // mmc5983ma_convert_packet(mmc5983ma_packet, &calibrated_packet);
      }
      osMutexRelease(sensorDataMutexHandle);
    } else {
      // TODO: error handling
    }
  }
}

void TaskWriteSerialData(void *argument) {}
