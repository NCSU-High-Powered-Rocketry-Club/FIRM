#include "firm_tasks.h"

#include "bmp581.h"
#include "logger.h"
#include "preprocessor.h"

void TaskCollectBarData(void* argument) {
    const TickType_t max_wait = MAX_WAIT_TIME(BMP_POLL_RATE_HZ);
    uint32_t notif_count = 0;

    for (;;) {
        notif_count = ulTaskNotifyTake(pdFALSE, max_wait);

        if (notif_count > 0) {
            BMP581Packet_t* bmp581_packet = logger_malloc_packet(sizeof(BMP581Packet_t));
            if (!bmp581_read_data(bmp581_packet)) {
                logger_write_entry('B', sizeof(BMP581Packet_t));
                // bmp581_convert_packet(bmp581_packet, &calibrated_packet);
            }
        } else {
            // todo: error handling
        }
    }
}

void TaskCollectIMUData(void* argument) {
    // for (;;) {}

    // ICM45686Packet_t* icm45686_packet = logger_malloc_packet(sizeof(ICM45686Packet_t));
    // if (!icm45686_read_data(icm45686_packet)) {
    //     logger_write_entry('I', sizeof(ICM45686Packet_t));
    //     // icm45686_convert_packet(icm45686_packet, &calibrated_packet);
    // }

}

void TaskCollectMagData(void* argument) {
    // for (;;) {}

    // MMC5983MAPacket_t* mmc5983ma_packet = logger_malloc_packet(sizeof(MMC5983MAPacket_t));
    // if (!mmc5983ma_read_data(mmc5983ma_packet, 0)) {
    //     logger_write_entry('M', sizeof(MMC5983MAPacket_t));
    //     // mmc5983ma_convert_packet(mmc5983ma_packet, &calibrated_packet);
    // }
}

void TaskWriteSerialData(void* argument) {
}
