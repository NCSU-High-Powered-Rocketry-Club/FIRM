#include "error_state_kalman_filter.h"
#include "eskf_functions.h"
#include "led.h"
#include "settings_manager.h"
#include <math.h>
#include <string.h>

/* ==================================================
 * Error-State Extended Kalman Filter (ESKF)
 * ================================================== */

/* ---- scratch buffers (static allocation, no malloc) --------------- */
#define N ESKF_ERROR_DIM
#define M ESKF_MEASUREMENT_DIM

static float R_imu_data[3 * 3];   /* IMU -> board rot matrix  */
static float R_mag_data[3 * 3];   /* board -> mag rot matrix  */
static float F_d_data[N * N];     /* discrete error Jacobian  */
static float Q_d_data[N * N];     /* discrete process noise   */
static float FP_data[N * N];      /* F @ P                    */
static float FP_FT_data[N * N];   /* F @ P @ F^T              */
static float HT_data[N * M];      /* H^T (5x4)                */
static float PHT_data[N * M];     /* P @ H^T (5x4)            */
static float HPHT_data[M * M];    /* H @ P @ H^T (4x4)        */
static float S_data[M * M];       /* S = HPHT + R             */
static float S_inv_data[M * M];   /* S^{-1}                   */
static float K_data[N * M];       /* Kalman gain (5x4)        */
static float KHP_data[N * N];     /* generic NxN update temp  */
static float temp_nn_data[N * N]; /* generic NxN temp         */

/* matrix_instance_f32 wrappers (set once, reused) */
static matrix_instance_f32 R_imu = {3, 3, R_imu_data};
static matrix_instance_f32 R_mag = {3, 3, R_mag_data};
static matrix_instance_f32 F_d = {N, N, F_d_data};
static matrix_instance_f32 Q_d = {N, N, Q_d_data};
static matrix_instance_f32 FP = {N, N, FP_data};
static matrix_instance_f32 FP_FT = {N, N, FP_FT_data};
static matrix_instance_f32 HT = {N, M, HT_data};
static matrix_instance_f32 PHT = {N, M, PHT_data};
static matrix_instance_f32 HPHT = {M, M, HPHT_data};
static matrix_instance_f32 S_mat = {M, M, S_data};
static matrix_instance_f32 S_inv = {M, M, S_inv_data};
static matrix_instance_f32 K_mat = {N, M, K_data};
static matrix_instance_f32 KHP = {N, N, KHP_data};

static float pressure_accum = 0.0F;
static float accel_accum[3] = {0.0F};
static float mag_accum[3] = {0.0F};
static uint32_t accum_count = 0;

static void copy_transpose_3x3(float destination[9], const float source[9]) {
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      destination[row * 3 + column] = source[column * 3 + row];
    }
  }
}

static void set_state_matrices(ESKF *eskf) {
  for (int i = 0; i < ESKF_ERROR_DIM; i++) {
    eskf->Q[i + i * ESKF_ERROR_DIM] = eskf_q_diag[i];
  }
  for (int i = 0; i < ESKF_MEASUREMENT_DIM; i++) {
    eskf->R[i + i * ESKF_MEASUREMENT_DIM] = eskf_r_diag[i];
  }
  for (int i = 0; i < ESKF_ERROR_DIM; i++) {
    eskf->P[i + i * ESKF_ERROR_DIM] = eskf_initial_cov_diag[i];
  }
}

int eskf_init(ESKF *eskf) {
  // zero everything first
  memset(eskf, 0, sizeof(ESKF));

  // Select rotation matrices based on hardware version
  const SystemSettings_t *settings = get_settings();
  if (settings->firmware_version[1] == '2') {
    // Firmware v2 uses the current PCB (FIRM hardware v1.0).
    memcpy(R_imu.pData, eskf_v2_R_imu_to_board, sizeof(R_imu_data));
    copy_transpose_3x3(R_mag.pData, eskf_v2_R_mag_to_board);
  } else {
    // Firmware v1 uses the legacy PCB (FIRM hardware v0.1).
    memcpy(R_imu.pData, eskf_v1_R_imu_to_board, sizeof(R_imu_data));
    copy_transpose_3x3(R_mag.pData, eskf_v1_R_mag_to_board);
  }

  // Copy initial nominal state (pos=0, vel=0, quat=identity)
  memcpy(eskf->x_nom, eskf_initial_state, sizeof(float) * ESKF_NOMINAL_DIM);

  // Initial pressure
  eskf->initial_pressure = pressure_accum / (float)accum_count;

  // Compute initial orientation from accel + mag, and set mag_world
  float initial_accel[3] = {accel_accum[0] / (float)accum_count,
                            accel_accum[1] / (float)accum_count,
                            accel_accum[2] / (float)accum_count};

  float initial_mag[3] = {mag_accum[0] / (float)accum_count, mag_accum[1] / (float)accum_count,
                          mag_accum[2] / (float)accum_count};
  memcpy(eskf->standby_acceleration, initial_accel, sizeof(initial_accel));
  memcpy(eskf->standby_magnetic_field, initial_mag, sizeof(initial_mag));
  eskf->acceleration_norm_reference =
      sqrtf(initial_accel[0] * initial_accel[0] + initial_accel[1] * initial_accel[1] +
            initial_accel[2] * initial_accel[2]);
  calculate_initial_orientation(initial_accel, initial_mag, R_imu.pData, R_mag.pData,
                                &eskf->x_nom[ESKF_QUAT_W], eskf->mag_world);

  // load Q/R/P diags
  set_state_matrices(eskf);
  eskf->pressure_reliability = 1.0F;
  eskf->pressure_coupling = 1.0F;
  // reset accumulated values
  accum_count = 0;
  pressure_accum = 0;
  memset(accel_accum, 0, sizeof(accel_accum));
  memset(mag_accum, 0, sizeof(mag_accum));
  return 0;
}

void eskf_accumulate(float pressure_raw, const float *accel_raw, const float *mag_raw) {
  for (int i = 0; i < 3; i++) {
    accel_accum[i] += accel_raw[i];
    mag_accum[i] += mag_raw[i];
  }
  pressure_accum += pressure_raw;
  accum_count++;
}

void eskf_predict(ESKF *eskf, const float u[ESKF_CONTROL_DIM], float dt) {
  if (dt < 1e-8F)
    return;
  eskf->last_dt_seconds = dt;

  /* A rocket can sit powered on for many minutes before launch.  Keep the
   * filter in a self-contained standby phase until motor acceleration is
   * unmistakable; no external flight-state input is required. */
  const float acceleration_norm =
      sqrtf(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
  const float angular_rate_norm =
      sqrtf(u[3] * u[3] + u[4] * u[4] + u[5] * u[5]);
  eskf->last_acceleration_norm = acceleration_norm;
  eskf->last_angular_rate_norm = angular_rate_norm;
  const uint8_t stationary_imu =
      acceleration_norm >= ESKF_STANDBY_ACCELERATION_MIN_G &&
      acceleration_norm <= ESKF_STANDBY_ACCELERATION_MAX_G &&
      angular_rate_norm <= ESKF_STANDBY_GYRO_MAX_DPS;
  eskf->standby_stationary = !eskf->launched && stationary_imu;
  if (eskf->standby_stationary || (eskf->landed && stationary_imu)) {
    const float imu_alpha =
        dt / (ESKF_STANDBY_IMU_TIME_CONSTANT + dt);
    for (int i = 0; i < 3; ++i) {
      eskf->standby_acceleration[i] +=
          imu_alpha * (u[i] - eskf->standby_acceleration[i]);
      eskf->gyro_bias[i] += imu_alpha * (u[i + 3] - eskf->gyro_bias[i]);
    }
    eskf->acceleration_norm_reference +=
        imu_alpha * (acceleration_norm - eskf->acceleration_norm_reference);
  }
  float corrected_u[ESKF_CONTROL_DIM];
  const float acceleration_scale =
      eskf->acceleration_norm_reference > 1e-6F
          ? 1.0F / eskf->acceleration_norm_reference
          : 1.0F;
  for (int i = 0; i < 3; ++i) {
    corrected_u[i] = u[i] * acceleration_scale;
    corrected_u[i + 3] = u[i + 3] - eskf->gyro_bias[i];
  }

  // normalize nominal quaternion state
  quaternion_normalize_f32(&eskf->x_nom[ESKF_QUAT_W]);

  // Linearize at the state being propagated, not at the end of the time step.
  eskf_error_jacobian(eskf->x_nom, corrected_u, dt, &R_imu, F_d_data);

  // propagate forward the nominal state
  const float velocity_before_prediction = eskf->x_nom[ESKF_VEL_Z];
  eskf_nominal_predict(eskf->x_nom, corrected_u, dt, &R_imu);

  /*
   * Dynamic pressure is least trustworthy during motor burn.  Identify coast
   * from sustained negative inertial acceleration so a small pressure-derived
   * velocity correction can be restored after burnout without contaminating
   * the boost estimate.
   */
  const float vertical_acceleration =
      (eskf->x_nom[ESKF_VEL_Z] - velocity_before_prediction) / dt;
  if (!eskf->launched) {
    const uint8_t launch_acceleration =
        acceleration_norm >= ESKF_LAUNCH_ACCELERATION_G &&
        vertical_acceleration >= ESKF_LAUNCH_MIN_VERTICAL_ACCELERATION;
    if (launch_acceleration) {
      if (!eskf->launch_candidate_active) {
        eskf->launch_candidate_active = 1U;
        eskf->launch_candidate_time_seconds = 0.0F;
        eskf->launch_candidate_velocity = 0.0F;
      }
      eskf->launch_candidate_time_seconds += dt;
      eskf->launch_candidate_velocity += vertical_acceleration * dt;
    } else {
      eskf->launch_candidate_active = 0U;
      eskf->launch_candidate_time_seconds = 0.0F;
      eskf->launch_candidate_velocity = 0.0F;
    }
  }
  if (eskf->coast_detected) {
    eskf->coast_time_seconds += dt;
  } else {
    if (eskf->x_nom[ESKF_VEL_Z] > ESKF_COAST_MIN_VELOCITY &&
        vertical_acceleration < ESKF_COAST_ENTER_ACCELERATION) {
      eskf->coast_time_seconds += dt;
    } else if (eskf->x_nom[ESKF_VEL_Z] <= ESKF_COAST_MIN_VELOCITY ||
               vertical_acceleration > ESKF_COAST_EXIT_ACCELERATION) {
      eskf->coast_time_seconds = 0.0F;
    }
    if (eskf->coast_time_seconds >= ESKF_COAST_CONFIRMATION_SECONDS) {
      eskf->coast_detected = 1U;
    }
  }
  if (eskf->coast_detected && !eskf->apogee_detected &&
      eskf->x_nom[ESKF_VEL_Z] <= 0.0F) {
    eskf->apogee_detected = 1U;
    eskf->apogee_altitude = eskf->x_nom[ESKF_POS_Z];
  }

  // Build discrete process noise Q_d = diag(qvar * dt)
  for (int i = 0; i < ESKF_ERROR_DIM; i++) {
    Q_d_data[i * ESKF_ERROR_DIM + i] = eskf->Q[i * ESKF_ERROR_DIM + i] * dt;
  }

  // error-state covariance propagation: P = F_d @ P @ F_d^T + Q_d
  matrix_instance_f32 P_mat = {N, N, eskf->P};
  matrix_instance_f32 F_dT = {N, N, temp_nn_data};

  mat_mult_f32(&F_d, &P_mat, &FP);   // FP = F_d @ P
  mat_trans_f32(&F_d, &F_dT);        // F_dT = F_d^T
  mat_mult_f32(&FP, &F_dT, &FP_FT);  // FP_FT = FP @ F_d^T
  mat_add_f32(&FP_FT, &Q_d, &P_mat); // P = FP_FT + Q_d
}

void eskf_update(ESKF *eskf) {
  /* Track local pad pressure until the filter itself detects launch.  This
   * avoids treating weather changes or a long prelaunch wait as altitude. */
  if (!eskf->launched && !eskf->launch_candidate_active && eskf->z[0] > 0.0F) {
    const float reference_alpha =
        eskf->last_dt_seconds /
        (ESKF_STANDBY_PRESSURE_TIME_CONSTANT + eskf->last_dt_seconds);
    eskf->initial_pressure += reference_alpha * (eskf->z[0] - eskf->initial_pressure);
  }

  // predicted measurement
  float z_pred[M];
  eskf_measurement_function(eskf->x_nom, eskf->initial_pressure, eskf->mag_world, &R_mag, z_pred);

  // measurement jacobian (4x5)
  float H_data[M * N] = {0};
  matrix_instance_f32 H = {M, N, H_data};
  eskf_measurement_jacobian(eskf->x_nom, eskf->initial_pressure, eskf->mag_world, R_mag.pData,
                            H_data);

  // Innovation y = z − z_pred
  float y[M];
  for (int i = 0; i < M; i++) {
    y[i] = eskf->z[i] - z_pred[i];
  }

  const float magnetometer_innovation_norm =
      sqrtf(y[1] * y[1] + y[2] * y[2] + y[3] * y[3]);
  if (eskf->launched && !eskf->apogee_detected &&
      magnetometer_innovation_norm > ESKF_MAGNETOMETER_MAX_DIRECTION_ERROR) {
    /* A large direction error is much more likely local magnetic interference
     * than a real attitude jump.  Remove the complete measurement rows before
     * forming S; they become eligible again automatically on the next sample. */
    memset(&H_data[N], 0, 3U * N * sizeof(float));
    y[1] = 0.0F;
    y[2] = 0.0F;
    y[3] = 0.0F;
  }

  // Innovation covariance: S = H @ P @ H^T + R
  matrix_instance_f32 P_mat = {N, N, eskf->P};
  matrix_instance_f32 R_mat = {M, M, eskf->R};

  mat_trans_f32(&H, &HT);             // HT = H^T (5x4)
  mat_mult_f32(&P_mat, &HT, &PHT);    // PHT = P @ H^T
  mat_mult_f32(&H, &PHT, &HPHT);      // HPHT = H @ PHT
  mat_add_f32(&HPHT, &R_mat, &S_mat); // S = HPHT + R
  mat_inverse_f32(&S_mat, &S_inv);    // S inverse (4x4)

  // Kalman gain: K = P @ H^T @ S^{-1}
  mat_mult_f32(&PHT, &S_inv, &K_mat); /* K = PHT @ S_inv */

  // Pressure decoupling: above threshold, pressure stops correcting altitude,
  // velocity, and quaternion states until coast is established.
  /* Dynamic-pressure decoupling is an ascent-only policy.  Using |velocity|
   * here creates a one-way failure during descent: once a bad estimate drops
   * below -20 m/s, pressure can no longer pull it back toward reality. */
  const float ascent_speed = fmaxf(eskf->x_nom[ESKF_VEL_Z], 0.0F);
  const float speed_coupling =
      1.0F /
      (1.0F + expf(ESKF_PV_COUPLING_SHARPNESS * (ascent_speed - ESKF_PV_COUPLING_SPEED)));
  const float coast_floor =
      eskf->coast_detected && eskf->x_nom[ESKF_VEL_Z] > 0.0F
          ? ESKF_COAST_PRESSURE_COUPLING *
                expf(-eskf->pressure_reliable_time_seconds /
                     ESKF_COAST_PRESSURE_COUPLING_TIME_CONSTANT)
          : 0.0F;
  const float velocity_coupling = coast_floor + (1.0F - coast_floor) * speed_coupling;
  const float position_coast_floor =
      eskf->coast_detected && eskf->x_nom[ESKF_VEL_Z] > 0.0F
          ? ESKF_COAST_PRESSURE_POSITION_COUPLING
          : 0.0F;
  const float position_coupling =
      position_coast_floor + (1.0F - position_coast_floor) * speed_coupling;

  /*
   * Detect disturbed static pressure from its implied altitude rate.  A
   * first-order altitude filter makes the rate robust to ordinary sample
   * noise.  Once disturbed, pressure is ignored while its local-altitude
   * offset changes, then faded back in when its rate becomes consistent.
   */
  const float pressure_ratio = eskf->z[0] / eskf->initial_pressure;
  const float pressure_altitude =
      pressure_ratio > 0.0F
          ? PRESSURE_ALTITUDE_CONST *
                (1.0F - powf(pressure_ratio, 1.0F / PRESSURE_EXPONENT))
          : eskf->filtered_pressure_altitude;
  const float altitude_filter_alpha =
      eskf->last_dt_seconds /
      (ESKF_PRESSURE_ALTITUDE_FILTER_TIME_CONSTANT + eskf->last_dt_seconds);
  const float previous_filtered_altitude = eskf->filtered_pressure_altitude;
  eskf->filtered_pressure_altitude +=
      altitude_filter_alpha * (pressure_altitude - eskf->filtered_pressure_altitude);
  const float pressure_altitude_velocity =
      (eskf->filtered_pressure_altitude - previous_filtered_altitude) / eskf->last_dt_seconds;
  const float pressure_velocity_error =
      fabsf(pressure_altitude_velocity - eskf->x_nom[ESKF_VEL_Z]);

  if (!eskf->launched && eskf->launch_candidate_active) {
    if (eskf->launch_candidate_time_seconds >= ESKF_LAUNCH_CONFIRMATION_SECONDS &&
        eskf->launch_candidate_velocity >= ESKF_LAUNCH_MIN_CANDIDATE_VELOCITY &&
        pressure_altitude >= ESKF_LAUNCH_PRESSURE_RISE_METERS) {
      /* Commit launch only when an inertial motor impulse and independent
       * pressure rise agree.  Short handling shocks can satisfy either signal
       * but cannot turn the filter into an unbounded flight solution. */
      eskf->launched = 1U;
      eskf->standby_stationary = 0U;
      eskf->x_nom[ESKF_POS_Z] =
          0.5F * eskf->launch_candidate_velocity * eskf->launch_candidate_time_seconds;
      eskf->x_nom[ESKF_VEL_Z] = eskf->launch_candidate_velocity;
      eskf->launch_candidate_active = 0U;
      eskf->coast_time_seconds = 0.0F;
      eskf->coast_detected = 0U;
      eskf->apogee_detected = 0U;
      eskf->landed = 0U;
    }
  }

  if (eskf->coast_detected) {
    if (!eskf->apogee_detected && eskf->x_nom[ESKF_VEL_Z] > 0.0F &&
        !eskf->pressure_disturbed &&
        pressure_velocity_error > ESKF_PRESSURE_DISTURBANCE_VELOCITY_ERROR) {
      eskf->pressure_disturbed = 1U;
      eskf->pressure_recovery_time_seconds = 0.0F;
      eskf->pressure_reliability = 0.0F;
      eskf->pressure_altitude_offset = pressure_altitude - eskf->x_nom[ESKF_POS_Z];
    }

    if (eskf->pressure_disturbed) {
      if (pressure_velocity_error <= ESKF_PRESSURE_RECOVERY_VELOCITY_ERROR) {
        eskf->pressure_recovery_time_seconds += eskf->last_dt_seconds;
      } else {
        /* Follow the changing local-pressure offset while the transient is in
         * progress.  Once its rate agrees with inertial velocity this value
         * freezes, preserving relative barometric motion without accepting
         * the altitude step. */
        eskf->pressure_altitude_offset = pressure_altitude - eskf->x_nom[ESKF_POS_Z];
        eskf->pressure_recovery_time_seconds = 0.0F;
        eskf->pressure_reliability = 0.0F;
      }

      if (eskf->pressure_recovery_time_seconds >=
          ESKF_PRESSURE_RECOVERY_CONFIRMATION_SECONDS) {
        eskf->pressure_reliability +=
            eskf->last_dt_seconds / ESKF_PRESSURE_RECOVERY_RAMP_SECONDS;
        if (eskf->pressure_reliability >= 1.0F) {
          eskf->pressure_reliability = 1.0F;
          eskf->pressure_disturbed = 0U;
        }
      }
    }

    if (eskf->pressure_disturbed || eskf->pressure_reliability < 1.0F ||
        fabsf(eskf->pressure_altitude_offset) > 1.0F) {
      eskf->pressure_reliable_time_seconds = 0.0F;
    } else {
      eskf->pressure_reliable_time_seconds += eskf->last_dt_seconds;
    }

    /* Fade a stable local-port offset slowly enough that it cannot create a
     * velocity step.  A transient is still tracked with zero measurement
     * weight, and the plausibility guard prevents a failed barometer from
     * dragging the state toward an unrelated absolute altitude. */
    if (!eskf->pressure_disturbed &&
        fabsf(pressure_altitude - eskf->x_nom[ESKF_POS_Z]) <=
            ESKF_PRESSURE_OFFSET_FADE_MAX_ALTITUDE_ERROR) {
      const float offset_fade_alpha =
          eskf->last_dt_seconds /
          (ESKF_PRESSURE_OFFSET_FADE_TIME_CONSTANT + eskf->last_dt_seconds);
      eskf->pressure_altitude_offset +=
          offset_fade_alpha * (0.0F - eskf->pressure_altitude_offset);
    }
  } else if (!eskf->launched) {
    eskf->pressure_disturbed = 0U;
    eskf->pressure_recovery_time_seconds = 0.0F;
    eskf->pressure_reliable_time_seconds = 0.0F;
    eskf->pressure_reliability = 1.0F;
    eskf->pressure_altitude_offset = 0.0F;
  }

  const float adjusted_pressure_altitude =
      pressure_altitude - eskf->pressure_altitude_offset;
  const float adjusted_pressure_base =
      1.0F - adjusted_pressure_altitude / PRESSURE_ALTITUDE_CONST;
  if (adjusted_pressure_base > 0.0F) {
    y[0] = eskf->initial_pressure *
               powf(adjusted_pressure_base, PRESSURE_EXPONENT) -
           z_pred[0];
  }

  const float pressure_altitude_error =
      fabsf(adjusted_pressure_altitude - eskf->x_nom[ESKF_POS_Z]);
  const float robust_pressure_weight =
      pressure_altitude_error > ESKF_PRESSURE_MAX_ALTITUDE_INNOVATION
          ? ESKF_PRESSURE_MAX_ALTITUDE_INNOVATION / pressure_altitude_error
          : 1.0F;
  const float pressure_measurement_weight =
      eskf->launched && !eskf->coast_detected
          ? 0.0F
          : eskf->pressure_reliability * robust_pressure_weight;
  for (int i = 0; i < N; i++) {
    K_data[i * M] *= pressure_measurement_weight;
  }
  eskf->pressure_coupling =
      velocity_coupling * pressure_measurement_weight;
  K_data[ESKF_DPOS_Z * M] *= position_coupling;
  for (int i = 1; i < N; i++) {
    K_data[i * M] *= velocity_coupling;
  }

  if (eskf->launched && !eskf->apogee_detected) {
    for (int row = ESKF_DTHETA_X; row <= ESKF_DTHETA_Z; ++row) {
      K_data[row * M] = 0.0F;
    }
  }

  // Error-state correction: dx = K @ y
  float dx[N];
  mat_vec_mult_f32(&K_mat, y, dx);

  // Inject error into nominal state
  eskf->x_nom[ESKF_POS_Z] += dx[ESKF_POS_Z];
  eskf->x_nom[ESKF_VEL_Z] += dx[ESKF_VEL_Z];
  if (eskf->apogee_detected) {
    /* The apogee latch is entered only after confirmed coast reaches zero
     * velocity.  Later positive velocity or a higher altitude is therefore a
     * pressure/attitude artifact, not a physically possible second ascent. */
    eskf->x_nom[ESKF_POS_Z] = fminf(eskf->x_nom[ESKF_POS_Z], eskf->apogee_altitude);
    eskf->x_nom[ESKF_VEL_Z] = fminf(eskf->x_nom[ESKF_VEL_Z], 0.0F);
  }

  // quaternion error injection (dx[2:5] = dtheta)
  float delta_q[4];
  rotvec_to_quat(&dx[ESKF_QUAT_W], delta_q);
  float new_q[4];
  quaternion_product_f32(&eskf->x_nom[ESKF_QUAT_W], delta_q, new_q);
  eskf->x_nom[ESKF_QUAT_W] = new_q[0];
  eskf->x_nom[ESKF_QUAT_X] = new_q[1];
  eskf->x_nom[ESKF_QUAT_Y] = new_q[2];
  eskf->x_nom[ESKF_QUAT_Z] = new_q[3];
  quaternion_normalize_f32(&eskf->x_nom[ESKF_QUAT_W]);

  /*
   * Joseph covariance update:
   *   P = (I - K H) P (I - K H)^T + K R K^T
   *
   * The pressure-decoupling policy above deliberately changes K after the
   * optimal Kalman gain is computed.  The abbreviated P - K H P update is
   * only equivalent for the unmodified optimal gain; using it here can make
   * P asymmetric or indefinite just before pressure recouples.
   */
  mat_mult_f32(&K_mat, &H, &KHP); // KHP used as K @ H
  for (int row = 0; row < N; ++row) {
    for (int column = 0; column < N; ++column) {
      FP_data[row * N + column] = (row == column ? 1.0F : 0.0F) - KHP_data[row * N + column];
    }
  }

  matrix_instance_f32 I_minus_KH = {N, N, FP_data};
  matrix_instance_f32 I_minus_KH_T = {N, N, temp_nn_data};
  mat_mult_f32(&I_minus_KH, &P_mat, &FP_FT); // FP_FT used as (I - K H) @ P
  mat_trans_f32(&I_minus_KH, &I_minus_KH_T);
  mat_mult_f32(&FP_FT, &I_minus_KH_T, &KHP); // KHP used as Joseph first term

  mat_mult_f32(&K_mat, &R_mat, &PHT); // PHT used as K @ R
  matrix_instance_f32 K_T = {M, N, HT_data};
  mat_trans_f32(&K_mat, &K_T);
  mat_mult_f32(&PHT, &K_T, &FP_FT); // FP_FT used as K @ R @ K^T
  mat_add_f32(&KHP, &FP_FT, &P_mat);

  /*
   * Reset the attitude-error tangent frame after injecting dtheta into the
   * right-multiplicative nominal quaternion:
   *   G_theta = I - 0.5 [dtheta]x
   *   P <- G P G^T
   */
  memset(FP_data, 0, sizeof(FP_data));
  for (int i = 0; i < N; ++i) {
    FP_data[i * N + i] = 1.0F;
  }
  const float half_dx = 0.5F * dx[ESKF_DTHETA_X];
  const float half_dy = 0.5F * dx[ESKF_DTHETA_Y];
  const float half_dz = 0.5F * dx[ESKF_DTHETA_Z];
  FP_data[ESKF_DTHETA_X * N + ESKF_DTHETA_Y] = half_dz;
  FP_data[ESKF_DTHETA_X * N + ESKF_DTHETA_Z] = -half_dy;
  FP_data[ESKF_DTHETA_Y * N + ESKF_DTHETA_X] = -half_dz;
  FP_data[ESKF_DTHETA_Y * N + ESKF_DTHETA_Z] = half_dx;
  FP_data[ESKF_DTHETA_Z * N + ESKF_DTHETA_X] = half_dy;
  FP_data[ESKF_DTHETA_Z * N + ESKF_DTHETA_Y] = -half_dx;

  matrix_instance_f32 reset_jacobian = {N, N, FP_data};
  matrix_instance_f32 reset_jacobian_T = {N, N, temp_nn_data};
  mat_mult_f32(&reset_jacobian, &P_mat, &FP_FT);
  mat_trans_f32(&reset_jacobian, &reset_jacobian_T);
  mat_mult_f32(&FP_FT, &reset_jacobian_T, &KHP);
  memcpy(eskf->P, KHP_data, sizeof(eskf->P));
  symmetrize(&P_mat);

  if (eskf->apogee_detected && !eskf->landed) {
    const uint8_t landed_imu =
        eskf->last_acceleration_norm >= ESKF_LANDED_ACCELERATION_MIN_G &&
        eskf->last_acceleration_norm <= ESKF_LANDED_ACCELERATION_MAX_G &&
        eskf->last_angular_rate_norm <= ESKF_LANDED_GYRO_MAX_DPS;
    const uint8_t landed_pressure =
        eskf->pressure_reliability >= 1.0F &&
        fabsf(pressure_altitude_velocity) <= ESKF_LANDED_MAX_PRESSURE_VELOCITY;
    const uint8_t landed_velocity =
        fabsf(eskf->x_nom[ESKF_VEL_Z]) <= ESKF_LANDED_MAX_ESTIMATED_VELOCITY;
    if (landed_imu && landed_pressure && landed_velocity) {
      eskf->landed_stationary_time_seconds += eskf->last_dt_seconds;
      if (eskf->landed_stationary_time_seconds >= ESKF_LANDED_CONFIRMATION_SECONDS) {
        eskf->landed = 1U;
      }
    } else {
      eskf->landed_stationary_time_seconds = 0.0F;
    }
  }
  if (eskf->landed) {
    /* Zero-velocity update after a sustained, independently stationary
     * measurement.  Clearing the associated covariance correlations keeps P
     * positive-definite while preventing a steady post-landing velocity bias. */
    eskf->x_nom[ESKF_VEL_Z] = 0.0F;
    for (int i = 0; i < N; ++i) {
      eskf->P[ESKF_DVEL_Z * N + i] = 0.0F;
      eskf->P[i * N + ESKF_DVEL_Z] = 0.0F;
    }
    eskf->P[ESKF_DVEL_Z * N + ESKF_DVEL_Z] = ESKF_LANDED_VELOCITY_VARIANCE;
  }

  /* Standby is a zero-altitude, zero-velocity constraint.  Reinitialize
   * attitude from the stationary gravity/magnetic averages and reset
   * covariance so a long pad wait cannot make launch artificially uncertain. */
  if (!eskf->launched) {
    eskf->x_nom[ESKF_POS_Z] = 0.0F;
    eskf->x_nom[ESKF_VEL_Z] = 0.0F;
    calculate_initial_orientation(eskf->standby_acceleration, eskf->standby_magnetic_field,
                                  R_imu.pData, R_mag.pData,
                                  &eskf->x_nom[ESKF_QUAT_W], eskf->mag_world);
    memset(eskf->P, 0, sizeof(eskf->P));
    for (int i = 0; i < N; ++i) {
      eskf->P[i * N + i] = eskf_initial_cov_diag[i];
    }
    eskf->coast_time_seconds = 0.0F;
    eskf->landed_stationary_time_seconds = 0.0F;
    eskf->coast_detected = 0U;
    eskf->apogee_detected = 0U;
    eskf->landed = 0U;
    eskf->apogee_altitude = 0.0F;
    eskf->filtered_pressure_altitude = 0.0F;
    eskf->pressure_altitude_offset = 0.0F;
    eskf->pressure_recovery_time_seconds = 0.0F;
    eskf->pressure_reliable_time_seconds = 0.0F;
    eskf->pressure_reliability = 1.0F;
    eskf->pressure_coupling = 1.0F;
    eskf->pressure_disturbed = 0U;
  }
}

void eskf_set_measurement(ESKF *eskf, const float *measurements) {
  /* measurements[0] = pressure
   * measurements[1..3] = raw magnetometer (not normalised) */
  eskf->z[0] = measurements[0];

  if (eskf->standby_stationary) {
    const float magnetic_alpha =
        eskf->last_dt_seconds /
        (ESKF_STANDBY_IMU_TIME_CONSTANT + eskf->last_dt_seconds);
    for (int i = 0; i < 3; ++i) {
      eskf->standby_magnetic_field[i] +=
          magnetic_alpha * (measurements[i + 1] - eskf->standby_magnetic_field[i]);
    }
  }

  /* normalise magnetometer */
  float mx = measurements[1], my = measurements[2], mz = measurements[3];
  float norm_mag = sqrtf(mx * mx + my * my + mz * mz);
  if (norm_mag < 1e-6F) {
    eskf->z[1] = 0.0F;
    eskf->z[2] = 0.0F;
    eskf->z[3] = 0.0F;
    return;
  }
  eskf->z[1] = mx / norm_mag;
  eskf->z[2] = my / norm_mag;
  eskf->z[3] = mz / norm_mag;
}

/* ---- helper: 3x3 matrix-vector multiply (row-major) --------------- */
static void mat3_vec3_mult(const float R[9], const float v[3], float out[3]) {
  out[0] = R[0] * v[0] + R[1] * v[1] + R[2] * v[2];
  out[1] = R[3] * v[0] + R[4] * v[1] + R[5] * v[2];
  out[2] = R[6] * v[0] + R[7] * v[1] + R[8] * v[2];
}

/* ---- helper: 3x3 transpose-vector multiply (R^T @ v) -------------- */
static void mat3T_vec3_mult(const float R[9], const float v[3], float out[3]) {
  out[0] = R[0] * v[0] + R[3] * v[1] + R[6] * v[2];
  out[1] = R[1] * v[0] + R[4] * v[1] + R[7] * v[2];
  out[2] = R[2] * v[0] + R[5] * v[1] + R[8] * v[2];
}

void calculate_initial_orientation(const float *imu_accel, const float *mag_field,
                                   const float *R_imu, const float *R_mag, float *init_quaternion,
                                   float *mag_world_frame) {
  /* Normalise raw readings */
  float norm_acc = sqrtf(imu_accel[0] * imu_accel[0] + imu_accel[1] * imu_accel[1] +
                         imu_accel[2] * imu_accel[2]);
  float norm_mag = sqrtf(mag_field[0] * mag_field[0] + mag_field[1] * mag_field[1] +
                         mag_field[2] * mag_field[2]);

  /* Normalise raw sensor readings */
  float acc_sensor_norm[3] = {imu_accel[0] / norm_acc, imu_accel[1] / norm_acc,
                              imu_accel[2] / norm_acc};
  float mag_sensor_norm[3] = {mag_field[0] / norm_mag, mag_field[1] / norm_mag,
                              mag_field[2] / norm_mag};

  /* Rotate accel from sensor → board frame */
  float acc_board[3];
  mat3_vec3_mult(R_imu, acc_sensor_norm, acc_board);

  /* Rotate mag from sensor → board frame using R_mag^T (inverse of board→sensor) */
  float mag_board_vec[3];
  mat3T_vec3_mult(R_mag, mag_sensor_norm, mag_board_vec);
  float mag_board[4] = {0.0F, mag_board_vec[0], mag_board_vec[1], mag_board_vec[2]};

  float roll = atan2f(acc_board[1], acc_board[2]);
  float pitch =
      atan2f(-acc_board[0], sqrtf(acc_board[1] * acc_board[1] + acc_board[2] * acc_board[2]));

  float cp = cosf(pitch), sp = sinf(pitch);
  float cr = cosf(roll), sr = sinf(roll);
  float mx2 = mag_board[1] * cp + mag_board[3] * sp;
  float my2 = mag_board[1] * sr * sp + mag_board[2] * cr - mag_board[3] * sr * cp;
  float yaw = atan2f(-my2, mx2);

  /* Euler → quaternion (ZYX convention) */
  float cr2 = cosf(roll * 0.5F), sr2 = sinf(roll * 0.5F);
  float cp2 = cosf(pitch * 0.5F), sp2 = sinf(pitch * 0.5F);
  float cy2 = cosf(yaw * 0.5F), sy2 = sinf(yaw * 0.5F);

  init_quaternion[0] = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
  init_quaternion[1] = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
  init_quaternion[2] = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
  init_quaternion[3] = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

  /* Rotate mag to world frame: q @ mag_board @ q_conj */
  float quat_conj[4] = {init_quaternion[0], -init_quaternion[1], -init_quaternion[2],
                        -init_quaternion[3]};
  float temp[4], mag_world[4];
  quaternion_product_f32(init_quaternion, mag_board, temp);
  quaternion_product_f32(temp, quat_conj, mag_world);
  mag_world_frame[0] = mag_world[1];
  mag_world_frame[1] = mag_world[2];
  mag_world_frame[2] = mag_world[3];
}
