#include "calibration.h"

#include "FreeRTOS.h"
#include "task.h"

static bool calibration_is_cancelled(const CommandContext_t* command_context) {
    if (command_context == NULL || command_context->is_cancelled == NULL) {
        return true;
    }
    return command_context->is_cancelled(command_context->cancel_context);
}

bool calibration_run_imu(const CommandContext_t* command_context) {
    int count = 0;
    while (!calibration_is_cancelled(command_context)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        count++;
        if (count >= 1000) {
            return true; // pretend we completed successfully after 10 seconds
        }
    }
    return false;
}

bool calibration_run_mag(const CommandContext_t* command_context) {
    int count = 0;
    while (!calibration_is_cancelled(command_context)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        count++;
        if (count >= 1000) {
            return true; // pretend we completed successfully after 10 seconds
        }
    }
    return false;
}
