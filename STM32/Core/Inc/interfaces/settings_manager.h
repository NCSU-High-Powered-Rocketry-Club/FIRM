#pragma once

#include "settings_storage.h"
#include "system_settings.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define FIRM_SETTINGS_FREQUENCY_MIN_HZ 1u
#define FIRM_SETTINGS_FREQUENCY_MAX_HZ 1000u
#define FIRM_FIRMWARE_VERSION "v2.1.0"

/**
 * @brief sets up the settings manager by ensuring the storage has existing settings and
 * calibration data. If existing, saves to settings struct
 * @note If calibration data does not exist, device needs to be given to repo owner to calibrate
 *
 * @retval 0 upon success
 */
int settings_manager_init();

/**
 * @brief Sets the calibration settings.
 * @note at least one of the calibration fields must be non-null
 *
 * @param accel_calibration desired accelerometer calibration (or null)
 * @param gyro_calibration desired gyroscope calibration (or null)
 * @param mag_calibration desired magnetometer calibration (or null)
 * @param high_g_calibration desired high-g accelerometer calibration (or null)
 */
bool settings_write_calibration(Calibration_t *accel_calibraion, Calibration_t *gyro_calibration,
                                Calibration_t *mag_calibration, Calibration_t *high_g_calibration);

/**
 * Writes the firm settings to the settings storage.
 *
 * @param firm_settings pointer to firm settings to write
 */
bool settings_write_firm_settings(SystemSettings_t *firm_settings);

/**
 * Reads the firm settings to the settings storage.
 *
 * @param firm_settings result of the stored firm settings
 */
bool settings_read_firm_settings(SystemSettings_t *firm_settings);

const SystemSettings_t *get_settings(void);