#include "error_state_kalman_filter.h"
#include "eskf_functions.h"
#include "settings.h"
#include <math.h>
#include <string.h>

/* ====================================================================
 * Error-State Extended Kalman Filter (ESKF) — core engine
 * ==================================================================== */

/* ---- scratch buffers (static allocation, no malloc) --------------- */
#define N ESKF_ERROR_DIM       /* 9  */
#define M ESKF_MEASUREMENT_DIM /* 4  */

static float F_d_data[N * N];        /* discrete error Jacobian  */
static float Q_d_data[N * N];        /* discrete process noise   */
static float FP_data[N * N];         /* F @ P                    */
static float FP_FT_data[N * N];      /* F @ P @ F^T              */
static float HT_data[N * M];         /* H^T (9×4)                */
static float PHT_data[N * M];        /* P @ H^T (9×4)            */
static float HPHT_data[M * M];       /* H @ P @ H^T (4×4)        */
static float S_data[M * M];          /* S = HPHT + R             */
static float S_inv_data[M * M];      /* S^{-1}                   */
static float K_data[N * M];          /* Kalman gain (9×4)        */
static float IKH_data[N * N];        /* I − K @ H                */
static float IKH_P_data[N * N];      /* (I−KH) @ P               */
static float IKH_P_IKHT_data[N * N]; /* (I−KH) @ P @ (I−KH)^T   */
static float IKHT_data[N * N];       /* (I−KH)^T                 */
static float KR_data[N * M];         /* K @ R (9×4)              */
static float KRKT_data[N * N];       /* K @ R @ K^T              */
static float KT_data[M * N];         /* K^T (4×9)                */
static float temp_nn_data[N * N];    /* generic N×N temp         */

/* matrix_instance_f32 wrappers (set once, reused) */
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
static matrix_instance_f32 IKH = {N, N, IKH_data};
static matrix_instance_f32 IKH_P = {N, N, IKH_P_data};
static matrix_instance_f32 IKHT = {N, N, IKHT_data};
static matrix_instance_f32 IKH_P_IKHT = {N, N, IKH_P_IKHT_data};
static matrix_instance_f32 KR = {N, M, KR_data};
static matrix_instance_f32 KT_mat = {M, N, KT_data};
static matrix_instance_f32 KRKT = {N, N, KRKT_data};

/* H lives on the stack as row-major float[M*N], wrapped when needed */

static void set_state_matrices(ESKF *eskf) {
  for (int i = 0; i < ESKF_ERROR_DIM; i++) {
    eskf->Q[i * ESKF_ERROR_DIM + i] = eskf_q_diag[i];
  }
  for (int i = 0; i < ESKF_MEASUREMENT_DIM; i++) {
    eskf->R[i * ESKF_MEASUREMENT_DIM + i] = eskf_r_diag[i];
  }
  for (int i = 0; i < ESKF_ERROR_DIM; i++) {
    eskf->P[i * ESKF_ERROR_DIM + i] = eskf_initial_cov_diag[i];
  }
}

/* ==================================================================== */

int eskf_init(ESKF *eskf) {
  /* Save accumulated data before zeroing (memset clears everything) */
  float saved_accel[3], saved_mag[3];
  float saved_pressure = eskf->pressure_accum;
  uint32_t saved_count = eskf->accum_count;
  memcpy(saved_accel, eskf->accel_accum, sizeof(saved_accel));
  memcpy(saved_mag, eskf->mag_accum, sizeof(saved_mag));

  /* Zero everything first */
  memset(eskf, 0, sizeof(ESKF));

  /* ---- Select rotation matrices based on hardware version ---- */
  if (firmSettings.firmware_version[0] == 'v' && firmSettings.firmware_version[1] == '1' &&
      firmSettings.firmware_version[2] == '.') {
    memcpy(eskf->R_imu_to_vehicle, eskf_v1_R_imu_to_vehicle, sizeof(eskf->R_imu_to_vehicle));
    memcpy(eskf->R_vehicle_to_mag, eskf_v1_R_vehicle_to_mag, sizeof(eskf->R_vehicle_to_mag));
  } else {
    /* Default: v2 (current hardware) */
    memcpy(eskf->R_imu_to_vehicle, eskf_v2_R_imu_to_vehicle, sizeof(eskf->R_imu_to_vehicle));
    memcpy(eskf->R_vehicle_to_mag, eskf_v2_R_vehicle_to_mag, sizeof(eskf->R_vehicle_to_mag));
  }

  /* Copy initial nominal state (pos=0, vel=0, quat=identity) */
  memcpy(eskf->x_nom, eskf_initial_state, sizeof(float) * ESKF_NOMINAL_DIM);

  /* Initial pressure */
  eskf->initial_pressure = saved_pressure / (float)saved_count;

  /* Compute initial orientation from accel + mag, and set mag_world */
  float initial_accel[3] = {saved_accel[0] / (float)saved_count,
                            saved_accel[1] / (float)saved_count,
                            saved_accel[2] / (float)saved_count};

  float initial_mag[3] = {saved_mag[0] / (float)saved_count,
                          saved_mag[1] / (float)saved_count,
                          saved_mag[2] / (float)saved_count};
  calculate_initial_orientation(initial_accel, initial_mag, eskf->R_imu_to_vehicle,
                                eskf->R_vehicle_to_mag, &eskf->x_nom[ESKF_QUAT_W],
                                eskf->mag_world);

  /* Initialise flight state to INIT + load Q/R diags */
  set_state_matrices(eskf);

  return 0;
}

/* ==================================================================== */

void eskf_accumulate(ESKF *eskf, float pressure_raw, const float *accel_raw, const float *mag_raw) {
  for (int i = 0; i < 3; i++) {
    eskf->accel_accum[i] += accel_raw[i];
    eskf->mag_accum[i] += mag_raw[i];
  }
  eskf->pressure_accum += pressure_raw;
  eskf->accum_count++;
}

/* ==================================================================== */

void eskf_predict(ESKF *eskf, const float u[ESKF_CONTROL_DIM], float dt) {
  if (dt < 1e-9F)
    return;

  /* ---- Normalise nominal quaternion ---- */
  quaternion_normalize_f32(&eskf->x_nom[ESKF_QUAT_W]);

  /* ---- Propagate nominal state ---- */
  eskf_nominal_predict(eskf->x_nom, u, dt, eskf->R_imu_to_vehicle);

  /* ---- Build discrete error Jacobian F_d ---- */
  eskf_error_jacobian(eskf->x_nom, u, dt, eskf->R_imu_to_vehicle, F_d_data);

  /* ---- Build discrete process noise Q_d = diag(qvar * dt) ---- */
  memset(Q_d_data, 0, sizeof(Q_d_data));
  for (int i = 0; i < ESKF_ERROR_DIM; i++) {
    Q_d_data[i * ESKF_ERROR_DIM + i] = eskf->Q[i * ESKF_ERROR_DIM + i] * dt;
  }

  /* ---- Covariance propagation: P = F_d @ P @ F_d^T + Q_d ---- */
  matrix_instance_f32 P_mat = {N, N, eskf->P};
  matrix_instance_f32 F_dT = {N, N, temp_nn_data};

  mat_mult_f32(&F_d, &P_mat, &FP);   /* FP = F_d @ P           */
  mat_trans_f32(&F_d, &F_dT);        /* F_dT = F_d^T           */
  mat_mult_f32(&FP, &F_dT, &FP_FT);  /* FP_FT = FP @ F_d^T     */
  mat_add_f32(&FP_FT, &Q_d, &P_mat); /* P = FP_FT + Q_d        */
}

/* ==================================================================== */

void eskf_update(ESKF *eskf, const float z[ESKF_MEASUREMENT_DIM]) {
  /* ---- Predicted measurement ---- */
  float z_pred[M];
  eskf_measurement_function(eskf->x_nom, eskf->initial_pressure, eskf->mag_world,
                            eskf->R_vehicle_to_mag, z_pred);

  /* ---- Measurement Jacobian H (4×9) ---- */
  float H_data[M * N];
  matrix_instance_f32 H = {M, N, H_data};
  eskf_measurement_jacobian(eskf->x_nom, eskf->initial_pressure, eskf->mag_world,
                            eskf->R_vehicle_to_mag, H_data);

  /* ---- Innovation y = z − z_pred ---- */
  float y[M];
  for (int i = 0; i < M; i++) {
    y[i] = z[i] - z_pred[i];
  }

  /* ---- Innovation covariance: S = H @ P @ H^T + R ---- */
  matrix_instance_f32 P_mat = {N, N, eskf->P};
  matrix_instance_f32 R_mat = {M, M, eskf->R};

  mat_trans_f32(&H, &HT);             /* HT = H^T (9×4) */
  mat_mult_f32(&P_mat, &HT, &PHT);    /* PHT = P @ H^T  */
  mat_mult_f32(&H, &PHT, &HPHT);      /* HPHT = H @ PHT */
  mat_add_f32(&HPHT, &R_mat, &S_mat); /* S = HPHT + R   */

  /* ---- S inverse (4×4) ---- */
  mat_inverse_f32(&S_mat, &S_inv);

  /* ---- Kalman gain: K = P @ H^T @ S^{-1} ---- */
  mat_mult_f32(&PHT, &S_inv, &K_mat); /* K = PHT @ S_inv */

  /* ---- Pressure→velocity decoupling sigmoid ---- */
  {
    float vx = eskf->x_nom[ESKF_VEL_X];
    float vy = eskf->x_nom[ESKF_VEL_Y];
    float vz = eskf->x_nom[ESKF_VEL_Z];
    float speed = sqrtf(vx * vx + vy * vy + vz * vz);
    float arg = ESKF_PV_COUPLING_SHARPNESS * (speed - ESKF_PV_COUPLING_SPEED);
    float coupling = 1.0F / (1.0F + expf(arg));

    /* Scale velocity rows (3,4,5) of pressure column (0) */
    K_data[3 * M + 0] *= coupling;
    K_data[4 * M + 0] *= coupling;
    K_data[5 * M + 0] *= coupling;
  }

  /* ---- Error-state correction: dx = K @ y ---- */
  float dx[N];
  mat_vec_mult_f32(&K_mat, y, dx);

  /* ---- Inject error into nominal state ---- */
  /* position + velocity: additive */
  for (int i = 0; i < 6; i++) {
    eskf->x_nom[i] += dx[i];
  }

  /* quaternion: multiplicative update q ← q * rotvec_to_quat(dθ) */
  {
    float dtheta[3] = {dx[6], dx[7], dx[8]};
    float delta_q[4];
    rotvec_to_quat(dtheta, delta_q);

    float quat[4] = {eskf->x_nom[ESKF_QUAT_W], eskf->x_nom[ESKF_QUAT_X], eskf->x_nom[ESKF_QUAT_Y],
                     eskf->x_nom[ESKF_QUAT_Z]};
    float new_q[4];
    quaternion_product_f32(quat, delta_q, new_q);
    eskf->x_nom[ESKF_QUAT_W] = new_q[0];
    eskf->x_nom[ESKF_QUAT_X] = new_q[1];
    eskf->x_nom[ESKF_QUAT_Y] = new_q[2];
    eskf->x_nom[ESKF_QUAT_Z] = new_q[3];
  }

  /* ---- Covariance update (Joseph form) ---- */
  /* P = (I − K@H) @ P @ (I − K@H)^T + K @ R @ K^T */
  {
    matrix_instance_f32 I_nn = {N, N, temp_nn_data};
    mat_set_identity_f32(&I_nn);    /* I(9)           */
    mat_mult_f32(&K_mat, &H, &IKH); /* IKH = K @ H    */
    mat_sub_f32(&I_nn, &IKH, &IKH); /* IKH = I − K@H  */

    mat_mult_f32(&IKH, &P_mat, &IKH_P);       /* IKH_P = IKH@P  */
    mat_trans_f32(&IKH, &IKHT);               /* IKHT           */
    mat_mult_f32(&IKH_P, &IKHT, &IKH_P_IKHT); /* (I-KH)P(I-KH)' */

    mat_mult_f32(&K_mat, &R_mat, &KR); /* KR = K@R       */
    mat_trans_f32(&K_mat, &KT_mat);    /* KT = K^T       */
    mat_mult_f32(&KR, &KT_mat, &KRKT); /* KRKT = KR@K^T  */

    mat_add_f32(&IKH_P_IKHT, &KRKT, &P_mat); /* P = ... + KRKT */
  }
}

/* ==================================================================== */

void eskf_set_measurement(ESKF *eskf, const float *measurements) {
  /* measurements[0] = pressure
   * measurements[1..3] = raw magnetometer (not normalised) */
  eskf->z[0] = measurements[0];

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

/* ==================================================================== */

/* ---- helper: 3×3 matrix-vector multiply (row-major) --------------- */
static void mat3_vec3_mult(const float R[9], const float v[3], float out[3]) {
  out[0] = R[0] * v[0] + R[1] * v[1] + R[2] * v[2];
  out[1] = R[3] * v[0] + R[4] * v[1] + R[5] * v[2];
  out[2] = R[6] * v[0] + R[7] * v[1] + R[8] * v[2];
}

/* ---- helper: 3×3 transpose-vector multiply (R^T @ v) -------------- */
static void mat3T_vec3_mult(const float R[9], const float v[3], float out[3]) {
  out[0] = R[0] * v[0] + R[3] * v[1] + R[6] * v[2];
  out[1] = R[1] * v[0] + R[4] * v[1] + R[7] * v[2];
  out[2] = R[2] * v[0] + R[5] * v[1] + R[8] * v[2];
}

void calculate_initial_orientation(const float *imu_accel, const float *mag_field,
                                   const float R_imu[9], const float R_mag[9],
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

  /* Rotate accel from sensor → vehicle frame */
  float acc_vehicle[3];
  mat3_vec3_mult(R_imu, acc_sensor_norm, acc_vehicle);

  /* Rotate mag from sensor → vehicle frame using R_mag^T (inverse of vehicle→sensor) */
  float mag_vehicle_vec[3];
  mat3T_vec3_mult(R_mag, mag_sensor_norm, mag_vehicle_vec);
  float mag_vehicle[4] = {0.0F, mag_vehicle_vec[0], mag_vehicle_vec[1], mag_vehicle_vec[2]};

  float roll = atan2f(acc_vehicle[1], acc_vehicle[2]);
  float pitch = atan2f(-acc_vehicle[0],
                       sqrtf(acc_vehicle[1] * acc_vehicle[1] + acc_vehicle[2] * acc_vehicle[2]));

  float cp = cosf(pitch), sp = sinf(pitch);
  float cr = cosf(roll), sr = sinf(roll);
  float mx2 = mag_vehicle[1] * cp + mag_vehicle[3] * sp;
  float my2 = mag_vehicle[1] * sr * sp + mag_vehicle[2] * cr - mag_vehicle[3] * sr * cp;
  float yaw = atan2f(-my2, mx2);

  /* Euler → quaternion (ZYX convention) */
  float cr2 = cosf(roll * 0.5F), sr2 = sinf(roll * 0.5F);
  float cp2 = cosf(pitch * 0.5F), sp2 = sinf(pitch * 0.5F);
  float cy2 = cosf(yaw * 0.5F), sy2 = sinf(yaw * 0.5F);

  init_quaternion[0] = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
  init_quaternion[1] = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
  init_quaternion[2] = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
  init_quaternion[3] = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

  /* Rotate mag to world frame: q @ mag_vehicle @ q_conj */
  float quat_conj[4] = {init_quaternion[0], -init_quaternion[1], -init_quaternion[2],
                        -init_quaternion[3]};
  float temp[4], mag_world[4];
  quaternion_product_f32(init_quaternion, mag_vehicle, temp);
  quaternion_product_f32(temp, quat_conj, mag_world);
  mag_world_frame[0] = mag_world[1];
  mag_world_frame[1] = mag_world[2];
  mag_world_frame[2] = mag_world[3];
}