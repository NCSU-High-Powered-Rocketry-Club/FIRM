#include "error_state_kalman_filter.h"

#include <stdint.h>
#include <string.h>

#if defined(_WIN32) || defined(__CYGWIN__)
#define FILTER_API __declspec(dllexport)
#else
#define FILTER_API
#endif

enum {
	SETUP_STATUS_OK = 0,
	SETUP_STATUS_INVALID_ARGUMENT = 1,
	SETUP_STATUS_NOT_INITIALIZED = 2,
	SETUP_STATUS_INIT_FAILED = 3,
	SETUP_STATUS_INSUFFICIENT_ACCUMULATION = 4,
};

enum {
	SETUP_ORIENTATION_V1 = 1,
	SETUP_ORIENTATION_V2 = 2,
};

static ESKF g_filter;
static uint32_t g_accumulated_samples = 0U;
static int g_initialized = 0;

FILTER_API int setup_reset(void) {
	memset(&g_filter, 0, sizeof(g_filter));
	g_accumulated_samples = 0U;
	g_initialized = 0;
	eskf_reset_accumulator();
	return SETUP_STATUS_OK;
}

FILTER_API int setup_set_orientation_variant(int orientation_variant) {
	SensorOrientations_t orientations;

	if (orientation_variant == SETUP_ORIENTATION_V1) {
		memcpy(orientations.R_imu_to_board, eskf_v1_R_imu_to_board, sizeof(orientations.R_imu_to_board));
		memcpy(orientations.R_mag_to_board, eskf_v1_R_mag_to_board, sizeof(orientations.R_mag_to_board));
	} else if (orientation_variant == SETUP_ORIENTATION_V2) {
		memcpy(orientations.R_imu_to_board, eskf_v2_R_imu_to_board, sizeof(orientations.R_imu_to_board));
		memcpy(orientations.R_mag_to_board, eskf_v2_R_mag_to_board, sizeof(orientations.R_mag_to_board));
	} else {
		return SETUP_STATUS_INVALID_ARGUMENT;
	}

	if (eskf_set_sensor_orientations(&orientations) != 0) {
		return SETUP_STATUS_INVALID_ARGUMENT;
	}
	return SETUP_STATUS_OK;
}

FILTER_API int setup_set_orientation_matrices(const float *r_imu_to_board,
																							const float *r_mag_to_board) {
	SensorOrientations_t orientations;

	if (r_imu_to_board == NULL || r_mag_to_board == NULL) {
		return SETUP_STATUS_INVALID_ARGUMENT;
	}

	memcpy(orientations.R_imu_to_board, r_imu_to_board, sizeof(orientations.R_imu_to_board));
	memcpy(orientations.R_mag_to_board, r_mag_to_board, sizeof(orientations.R_mag_to_board));

	if (eskf_set_sensor_orientations(&orientations) != 0) {
		return SETUP_STATUS_INVALID_ARGUMENT;
	}
	return SETUP_STATUS_OK;
}

FILTER_API int setup_accumulate_sample(float pressure_pa, float accel_x_g, float accel_y_g,
																			 float accel_z_g, float mag_x_ut, float mag_y_ut,
																			 float mag_z_ut) {
	const float accel[3] = {accel_x_g, accel_y_g, accel_z_g};
	const float mag[3] = {mag_x_ut, mag_y_ut, mag_z_ut};

	eskf_accumulate(pressure_pa, accel, mag);
	g_accumulated_samples++;
	return SETUP_STATUS_OK;
}

FILTER_API int setup_initialize(void) {
	if (g_accumulated_samples == 0U) {
		return SETUP_STATUS_INSUFFICIENT_ACCUMULATION;
	}

	if (eskf_init(&g_filter) != 0) {
		g_initialized = 0;
		return SETUP_STATUS_INIT_FAILED;
	}

	g_initialized = 1;
	g_accumulated_samples = 0U;
	return SETUP_STATUS_OK;
}

FILTER_API int setup_initialize_default(uint32_t sample_count) {
	uint32_t i;
	const float accel_rest[3] = {0.0F, 0.0F, 1.0F};
	const float mag_rest[3] = {20.0F, 0.0F, 45.0F};
	const float pressure_rest = 101325.0F;

	if (sample_count == 0U) {
		return SETUP_STATUS_INVALID_ARGUMENT;
	}

	for (i = 0U; i < sample_count; i++) {
		eskf_accumulate(pressure_rest, accel_rest, mag_rest);
	}
	g_accumulated_samples += sample_count;

	return setup_initialize();
}

FILTER_API int setup_step(float dt_s, float pressure_pa, float accel_x_g, float accel_y_g,
													float accel_z_g, float gyro_x_dps, float gyro_y_dps,
													float gyro_z_dps, float mag_x_ut, float mag_y_ut,
													float mag_z_ut) {
	float u[ESKF_CONTROL_DIM];
	float z[ESKF_MEASUREMENT_DIM];

	if (!g_initialized) {
		return SETUP_STATUS_NOT_INITIALIZED;
	}
	if (dt_s <= 0.0F) {
		return SETUP_STATUS_INVALID_ARGUMENT;
	}

	u[0] = accel_x_g;
	u[1] = accel_y_g;
	u[2] = accel_z_g;
	u[3] = gyro_x_dps;
	u[4] = gyro_y_dps;
	u[5] = gyro_z_dps;

	z[0] = pressure_pa;
	z[1] = mag_x_ut;
	z[2] = mag_y_ut;
	z[3] = mag_z_ut;

	eskf_predict(&g_filter, u, dt_s);
	eskf_set_measurement(&g_filter, z);
	eskf_update(&g_filter);
	return SETUP_STATUS_OK;
}

FILTER_API int setup_get_state(float *altitude_m, float *velocity_z_mps, float *quat_w,
															 float *quat_x, float *quat_y, float *quat_z) {
	if (!g_initialized) {
		return SETUP_STATUS_NOT_INITIALIZED;
	}
	if (altitude_m == NULL || velocity_z_mps == NULL || quat_w == NULL || quat_x == NULL ||
			quat_y == NULL || quat_z == NULL) {
		return SETUP_STATUS_INVALID_ARGUMENT;
	}

	*altitude_m = g_filter.x_nom[ESKF_POS_Z];
	*velocity_z_mps = g_filter.x_nom[ESKF_VEL_Z];
	*quat_w = g_filter.x_nom[ESKF_QUAT_W];
	*quat_x = g_filter.x_nom[ESKF_QUAT_X];
	*quat_y = g_filter.x_nom[ESKF_QUAT_Y];
	*quat_z = g_filter.x_nom[ESKF_QUAT_Z];
	return SETUP_STATUS_OK;
}

FILTER_API int setup_is_initialized(void) { return g_initialized; }
