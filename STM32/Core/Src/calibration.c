#include "calibration.h"

#include "FreeRTOS.h"
#include "task.h"

static bool calibration_is_cancelled(const CommandContext_t* ctx) {
    if (ctx == NULL || ctx->is_cancelled == NULL) {
        return true;
    }
    return ctx->is_cancelled(ctx->user);
}

bool calibration_run_imu(const CommandContext_t* ctx) {
    int count = 0;
    while (!calibration_is_cancelled(ctx)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        count++;
        if (count >= 1000) {
            return true; // pretend we completed successfully after 10 seconds
        }
    }
    return false;
}

bool calibration_run_mag(const CommandContext_t* ctx) {
    int count = 0;
    while (!calibration_is_cancelled(ctx)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        count++;
        if (count >= 1000) {
            return true; // pretend we completed successfully after 10 seconds
        }
    }
    return false;
}
