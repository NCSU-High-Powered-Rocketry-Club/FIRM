#include "error_state_kalman_filter.h"
#include "eskf_functions.h"
#include "firm_fsm.h"
#include "led.h"
#include "settings.h"
#include <math.h>
#include <string.h>

/* ==================================================
 * Error-State Extended Kalman Filter (ESKF)
 * ==================================================
 * Nominal state (6): altitude(1), velocity(1), quat(4) [w,x,y,z]
 * Error   state (5): δalt(1), δvel(1), δθ(3)
 * Measurement   (1): pressure(1) only
 * Control       (6): accel(3), gyro(3) (sensor frame, g and deg/s)
 * ================================================== */

/* ---- scratch buffers (static allocation, no malloc) --------------- */
#define N ESKF_ERROR_DIM
#define M ESKF_MEASUREMENT_DIM

static float R_imu_data[3 * 3];      /* IMU -> board rot matrix  */
static float R_mag_data[3 * 3];      /* mag -> board rot matrix  */
static float F_d_data[N * N];        /* discrete error Jacobian  */
static float Q_d_data[N * N];        /* discrete process noise   */
static float FP_data[N * N];         /* F @ P                    */
static float FP_FT_data[N * N];      /* F @ P @ F^T              */
static float HT_data[N * M];         /* H^T (5x4)                */
static float PHT_data[N * M];        /* P @ H^T (5x4)            */
static float HPHT_data[M * M];       /* H @ P @ H^T (4x4)        */
static float S_data[M * M];          /* S = HPHT + R             */
static float S_inv_data[M * M];      /* S^{-1}                   */
static float K_data[N * M];          /* Kalman gain (5x4)        */
static float HP_data[M * N];         /* H @ P                    */
static float KHP_data[N * N];        /* K @ (H @ P)              */
static float temp_nn_data[N * N];    /* generic NxN temp         */

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
static matrix_instance_f32 HP = {M, N, HP_data};
static matrix_instance_f32 KHP = {N, N, KHP_data};

static float pressure_accum = 0.0F;
static float accel_accum[3] = {0.0F};
static float mag_accum[3] = {0.0F};
static uint32_t accum_count = 0;

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
  if (firmSettings.firmware_version[1] == '2') {
    // firmware version v1.x.x (hardware v0.1), legacy PCB version
    memcpy(R_imu.pData, eskf_v2_R_imu_to_board, sizeof(R_imu_data));
    memcpy(R_mag.pData, eskf_v2_R_mag_to_board, sizeof(R_mag_data));
  } else {
    // firmware version v2.x.x (hardware v1.0), current PCB version
    memcpy(R_imu.pData, eskf_v1_R_imu_to_board, sizeof(R_imu_data));
    memcpy(R_mag.pData, eskf_v1_R_mag_to_board, sizeof(R_mag_data));
  }

  // Copy initial nominal state (pos=0, vel=0, quat=identity)
  memcpy(eskf->x_nom, eskf_initial_state, sizeof(float) * ESKF_NOMINAL_DIM);

  // Initial pressure
  eskf->initial_pressure = pressure_accum / (float)accum_count;

  // Compute initial orientation from accel + mag, and set mag_world
  float initial_accel[3] = {accel_accum[0] / (float)accum_count,
                            accel_accum[1] / (float)accum_count,
                            accel_accum[2] / (float)accum_count};

  float initial_mag[3] = {mag_accum[0] / (float)accum_count,
                          mag_accum[1] / (float)accum_count,
                          mag_accum[2] / (float)accum_count};
  calculate_initial_orientation(initial_accel, initial_mag, R_imu.pData,
                                R_mag.pData, &eskf->x_nom[ESKF_QUAT_W],
                                eskf->mag_world);

  // load Q/R/P diags
  set_state_matrices(eskf);
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

  // normalize nominal quaternion state
  quaternion_normalize_f32(&eskf->x_nom[ESKF_QUAT_W]);

  // propagate forward the nominal state
  eskf_nominal_predict(eskf->x_nom, u, dt, &R_imu);

  // build the error jacobian
  eskf_error_jacobian(eskf->x_nom, u, dt, &R_imu, F_d_data);

  // Build discrete process noise Q_d = diag(qvar * dt)
  for (int i = 0; i < ESKF_ERROR_DIM; i++) {
    Q_d_data[i * ESKF_ERROR_DIM + i] = eskf->Q[i * ESKF_ERROR_DIM + i] * dt;
  }

  // error-state covariance propagation: P = F_d @ P @ F_d^T + Q_d
  matrix_instance_f32 P_mat = {N, N, eskf->P};
  matrix_instance_f32 F_dT = {N, N, temp_nn_data};

  mat_mult_f32(&F_d, &P_mat, &FP); // FP = F_d @ P
  mat_trans_f32(&F_d, &F_dT); // F_dT = F_d^T
  mat_mult_f32(&FP, &F_dT, &FP_FT); // FP_FT = FP @ F_d^T
  mat_add_f32(&FP_FT, &Q_d, &P_mat); // P = FP_FT + Q_d
}

void eskf_update(ESKF *eskf) {
  // predicted measurement
  float z_pred[M];
  eskf_measurement_function(eskf->x_nom, eskf->initial_pressure, eskf->mag_world,
                            &R_mag, z_pred);

  // measurement jacobian (4x5)
  float H_data[M * N] = {0};
  matrix_instance_f32 H = {M, N, H_data};
  eskf_measurement_jacobian(eskf->x_nom, eskf->initial_pressure, eskf->mag_world,
                            R_mag.pData, H_data);

  // Innovation y = z − z_pred
  float y[M];
  for (int i = 0; i < M; i++) {
    y[i] = eskf->z[i] - z_pred[i];
  }

  // Innovation covariance: S = H @ P @ H^T + R
  matrix_instance_f32 P_mat = {N, N, eskf->P};
  matrix_instance_f32 R_mat = {M, M, eskf->R};

  mat_trans_f32(&H, &HT); // HT = H^T (5x4)
  mat_mult_f32(&P_mat, &HT, &PHT); // PHT = P @ H^T
  mat_mult_f32(&H, &PHT, &HPHT); // HPHT = H @ PHT
  mat_add_f32(&HPHT, &R_mat, &S_mat); // S = HPHT + R
  mat_inverse_f32(&S_mat, &S_inv); // S inverse (4x4)

  // Kalman gain: K = P @ H^T @ S^{-1}
  mat_mult_f32(&PHT, &S_inv, &K_mat); /* K = PHT @ S_inv */

  // pressure decoupling: above threshold, pressure stops correcting velocity
  // and quaternion states
  float speed = fabsf(eskf->x_nom[ESKF_VEL_Z]);
  float coupling = 1.0F / (1.0F + expf(ESKF_PV_COUPLING_SHARPNESS * (speed - ESKF_PV_COUPLING_SPEED)));
  for (int i = 1; i < N; i++) {
    K_data[i * M] *= coupling; // decouple velocity and quat component from pressure measurement
  }

  // Error-state correction: dx = K @ y
  float dx[N];
  mat_vec_mult_f32(&K_mat, y, dx);

  // Inject error into nominal state
  eskf->x_nom[ESKF_POS_Z] += dx[ESKF_POS_Z];
  eskf->x_nom[ESKF_VEL_Z] += dx[ESKF_VEL_Z];

  // quaternion error injection (dx[2:5] = dtheta)
  float delta_q[4];
  rotvec_to_quat(&dx[ESKF_QUAT_W], delta_q);
  float new_q[4];
  quaternion_product_f32(&eskf->x_nom[ESKF_QUAT_W], delta_q, new_q);
  eskf->x_nom[ESKF_QUAT_W] = new_q[0];
  eskf->x_nom[ESKF_QUAT_X] = new_q[1];
  eskf->x_nom[ESKF_QUAT_Y] = new_q[2];
  eskf->x_nom[ESKF_QUAT_Z] = new_q[3];

  // Covariance Update: P = P - K @ (H @ P)
  mat_mult_f32(&H, &P_mat, &HP);
  mat_mult_f32(&K_mat, &HP, &KHP);
  mat_sub_f32(&P_mat, &KHP, &P_mat);
  symmetrize(&P_mat);
}

void eskf_set_measurement(ESKF *eskf, const float *measurements) {
  /* measurements[0] = pressure */
  eskf->z[0] = measurements[0];
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
                                   const float *R_imu, const float *R_mag,
                                   float *init_quaternion, float *mag_world_frame) {
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
  float pitch = atan2f(-acc_board[0],
                       sqrtf(acc_board[1] * acc_board[1] + acc_board[2] * acc_board[2]));

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