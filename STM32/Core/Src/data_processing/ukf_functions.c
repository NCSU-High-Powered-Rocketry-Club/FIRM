#include "ukf_functions.h"


void ukf_state_transition_function(const float *sigmas, const float dt, const State state, float *prediction) {
    memset(prediction, 0, sizeof(float) * UKF_STATE_DIMENSION);

    // get quaternion elements and normalize
    float quat[4] = {sigmas[18], sigmas[19], sigmas[20], sigmas[21]};
    quaternion_normalize_f32(quat);

    // copy over states 7 - 21
    memcpy(&prediction[6], &sigmas[6], sizeof(float) * (UKF_STATE_DIMENSION - 6));

    if (state == 1 || state == 2 || state == 3) {
        // calculate next quaternion
        float delta_theta[3] = {
            sigmas[9] * dt,
            sigmas[10] * dt,
            sigmas[11] * dt
        };

        float delta_quat[4];
        rotvec_to_quat(delta_theta, delta_quat);
        // next quat = quat * delta_q
        float next_q[4];
        quaternion_product_f32(quat, delta_quat, next_q);
        memcpy(&prediction[18], next_q, sizeof(float) * 4);
        

        // calculate next velocity and position
        float accel_grav[3] = {
            sigmas[6] * GRAVITY_METERS_PER_SECOND_SQUARED,
            sigmas[7] * GRAVITY_METERS_PER_SECOND_SQUARED,
            sigmas[8] * GRAVITY_METERS_PER_SECOND_SQUARED
        };
        // subtract off gravity in z direction
        accel_grav[2] -= GRAVITY_METERS_PER_SECOND_SQUARED;

        // velocity update
        prediction[3] = sigmas[3] + accel_grav[0] * dt;
        prediction[4] = sigmas[4] + accel_grav[1] * dt;
        prediction[5] = sigmas[5] + accel_grav[2] * dt;

        // drag model
        if (prediction[5] > UKF_MIN_VEL_FOR_DRAG) {
            float drag_acc = 0.5F * DRAG_PARAM * prediction[5] * prediction[5];
            prediction[5] += drag_acc * dt;
        }

        // position update
        prediction[0] = sigmas[0] + sigmas[3] * dt;
        prediction[1] = sigmas[1] + sigmas[4] * dt;
        prediction[2] = sigmas[2] + sigmas[5] * dt;

        return;
    }

    if (state == 4) {
        // landed state
        float grav_vec[3] = {0.0F, 0.0F, GRAVITY_METERS_PER_SECOND_SQUARED};

        prediction[3] = sigmas[3] + (sigmas[6] * GRAVITY_METERS_PER_SECOND_SQUARED - grav_vec[0]) * dt;
        prediction[4] = sigmas[4] + (sigmas[7] * GRAVITY_METERS_PER_SECOND_SQUARED - grav_vec[1]) * dt;
        prediction[5] = sigmas[5] + (sigmas[8] * GRAVITY_METERS_PER_SECOND_SQUARED - grav_vec[2]) * dt;

        // heavy damping in landed state
        prediction[3] = sigmas[3] * 1e-2F;
        prediction[4] = sigmas[4] * 1e-2F;

        // position update
        prediction[0] = sigmas[0] + sigmas[3] * dt;
        prediction[1] = sigmas[1] + sigmas[4] * dt;
        prediction[2] = sigmas[2] + sigmas[5] * dt;

        return;
    }

    if (state == 0) {
        // standby state, set position and velocity to 0
        memset(&prediction[0], 0, sizeof(float) * 6);

        // damping the gyro prediction to drive to 0
        prediction[9] = sigmas[9] * 0.5F;
        prediction[10] = sigmas[10] * 0.5F;
        prediction[11] = sigmas[11] * 0.5F;

        return;
    }

}

void ukf_measurement_function(const float *sigmas, const UKF *ukfh, float *measurement_sigmas) {
    measurement_sigmas[0] = ukfh->initial_pressure * powf((1.0F - (sigmas[2] / 44330.0F)), 5.255876F);

    // normalize quaternions
    float quat_state[4];
    memcpy(quat_state, &sigmas[UKF_STATE_DIMENSION - 4], sizeof(quat_state));
    quaternion_normalize_f32(quat_state);

    // conjugate of quaternion state
    float quat_state_conj[4] = {
        quat_state[0],
        -quat_state[1],
        -quat_state[2],
        -quat_state[3],
    };

    // global acceleration sigma state as quaternion
    float global_accel[4] = {0.0F, sigmas[6], sigmas[7], sigmas[8]};

    // Rotate to vehicle frame
    float temp_acc[4];
    quaternion_product_f32(quat_state_conj, global_accel, temp_acc);
    float acc_vehicle[4];
    quaternion_product_f32(temp_acc, quat_state, acc_vehicle);

    // rotate vehicle frame accel 45 degrees ccw to align with how imu is mounted on FIRM board
    float acc_vx = acc_vehicle[1];
    float acc_vy = acc_vehicle[2];
    float acc_vz = acc_vehicle[3];
    measurement_sigmas[1] = (acc_vx / SQRT2_F + acc_vy / SQRT2_F) + sigmas[12];  // accel X
    measurement_sigmas[2] = (-acc_vx / SQRT2_F + acc_vy / SQRT2_F) + sigmas[13]; // accel Y
    measurement_sigmas[3] = acc_vz + sigmas[14]; // accel Z

    // clip accel X and accel Y
    float acc_clip = 19.2882F;
    if (fabsf(measurement_sigmas[1]) > acc_clip) {
        measurement_sigmas[1] = (measurement_sigmas[1] > 0.0F) ? acc_clip : -acc_clip;
    }
    float acc_y_clip = 19.6925F;
    if (fabsf(measurement_sigmas[2]) > acc_y_clip) {
        measurement_sigmas[2] = (measurement_sigmas[2] > 0.0F) ? acc_y_clip : -acc_y_clip;
    }

    // vehicle frame gyro in degrees per second (no global gyro because it is defined in the state
    // vector as vehicle-frame, not global-frame like acceleration)
    float vehicle_gyro[3];
    for (int i = 0; i < 3; ++i) {
        vehicle_gyro[i] = sigmas[9 + i] * (180.0F / PI_F);
    }
    // 45-degree rotation and add biases
    float gyro_gx = vehicle_gyro[0];
    float gyro_gy = vehicle_gyro[1];
    float gyro_gz = vehicle_gyro[2];
    measurement_sigmas[4] = (gyro_gx / SQRT2_F + gyro_gy / SQRT2_F) + sigmas[15]; // gyro x
    measurement_sigmas[5] = (-gyro_gx / SQRT2_F + gyro_gy / SQRT2_F) + sigmas[16]; // gyro y
    measurement_sigmas[6] = gyro_gz + sigmas[17]; // gyro z

    // Magnetometer
    float mag_world_q[4] = {
        0.0F,
        ukfh->mag_world[0],
        ukfh->mag_world[1],
        ukfh->mag_world[2]
    };

    // Rotate mag_world to vehicle frame: conj * mag_world_q * quat
    float temp_mag[4];
    quaternion_product_f32(quat_state_conj, mag_world_q, temp_mag);
    float mag_vehicle[4];
    quaternion_product_f32(temp_mag, quat_state, mag_vehicle);

    // Apply R_vehicle_to_mag: [x, y, -z]
    float mag_vx = mag_vehicle[1];
    float mag_vy = mag_vehicle[2];
    float mag_vz = mag_vehicle[3];
    measurement_sigmas[7] = mag_vx;  // mag_sensor x
    measurement_sigmas[8] = mag_vy;  // y
    measurement_sigmas[9] = -mag_vz; // z (flipped)
}

