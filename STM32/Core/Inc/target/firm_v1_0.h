#pragma once
#include "main.h"
#include "settings_storage.h"
#include "targets.h"
#include "w25q128jv.h"

#if FIRM_HARDWARE_VERSION == VERSION_V1_0

/** Chip selects, settings flash and sensor bring-up. Must finish before any FreeRTOS call. */
int firm_init_hardware(void);

/** SD card mount and logger storage setup. Call from a task, after osKernelStart(). */
int firm_init_log_storage(void);

#endif
