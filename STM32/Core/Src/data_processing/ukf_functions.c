#include "ukf_functions.h"

static const double pi = 3.141592653589793;

void ukf_state_transition_function(const double *sigmas, const double dt, const int state, double *prediction) {
    memset(prediction, 0, sizeof(double) * UKF_STATE_DIMENSION);

    // get quaternion elements and normalize
    double quat[4] = {sigmas[18], sigmas[19], sigmas[20], sigmas[21]};
    quaternion_normalize_f64(quat);

    // copy over states 7 - 21
    memcpy(&prediction[6], &sigmas[6], sizeof(double) * (UKF_STATE_DIMENSION - 6));

    if (state == 1 || state == 2 || state == 3) {
        // calculate next quaternion
        double delta_theta[3] = {
            sigmas[9] * dt,
            sigmas[10] * dt,
            sigmas[11] * dt
        };

        double delta_quat[4];
        rotvec_to_quat(delta_theta, delta_quat);
        // next quat = quat * delta_q
        double next_q[4];
        quaternion_product_f64(quat, delta_quat, next_q);
        memcpy(&prediction[18], next_q, sizeof(double) * 4);
        

        // calculate next velocity and position
        double accel_grav[3] = {
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
            double drag_acc = 0.5 * DRAG_PARAM * prediction[5] * prediction[5];
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
        double grav_vec[3] = {0.0, 0.0, GRAVITY_METERS_PER_SECOND_SQUARED};

        prediction[3] = sigmas[3] + (sigmas[6] * GRAVITY_METERS_PER_SECOND_SQUARED - grav_vec[0]) * dt;
        prediction[4] = sigmas[4] + (sigmas[7] * GRAVITY_METERS_PER_SECOND_SQUARED - grav_vec[1]) * dt;
        prediction[5] = sigmas[5] + (sigmas[8] * GRAVITY_METERS_PER_SECOND_SQUARED - grav_vec[2]) * dt;

        // heavy damping in landed state
        prediction[3] = sigmas[3] * 1e-2;
        prediction[4] = sigmas[4] * 1e-2;

        // position update
        prediction[0] = sigmas[0] + sigmas[3] * dt;
        prediction[1] = sigmas[1] + sigmas[4] * dt;
        prediction[2] = sigmas[2] + sigmas[5] * dt;

        return;
    }

    if (state == 0) {
        // standby state, set position and velocity to 0
        memset(&prediction[0], 0, sizeof(double) * 6);

        // damping the gyro prediction to drive to 0
        prediction[9] = sigmas[9] * 0.5;
        prediction[10] = sigmas[10] * 0.5;
        prediction[11] = sigmas[11] * 0.5;

        return;
    }

}

void ukf_measurement_function(const double *sigmas, const UKF *ukfh, double *measurement_sigmas) {
    measurement_sigmas[0] = ukfh->initial_pressure * pow((1.0 - (sigmas[2] / 44330.0)), 5.255876);

    // normalize quaternions
    double quat_state[4];
    memcpy(quat_state, &sigmas[UKF_STATE_DIMENSION - 4], sizeof(quat_state));
    quaternion_normalize_f64(quat_state);

    // conjugate of quaternion state
    double quat_state_conj[4] = {
        quat_state[0],
        -quat_state[1],
        -quat_state[2],
        -quat_state[3],
    };

    // global acceleration sigma state as quaternion
    double global_accel[4] = {0.0, sigmas[6], sigmas[7], sigmas[8]};

    // Rotate to vehicle frame
    double temp_acc[4];
    quaternion_product_f64(quat_state_conj, global_accel, temp_acc);
    double acc_vehicle[4];
    quaternion_product_f64(temp_acc, quat_state, acc_vehicle);

    // rotate vehicle frame accel 45 degrees ccw to align with how imu is mounted on FIRM board
    double sqrt2 = sqrt(2.0);
    double acc_vx = acc_vehicle[1];
    double acc_vy = acc_vehicle[2];
    double acc_vz = acc_vehicle[3];
    measurement_sigmas[1] = (acc_vx / sqrt2 + acc_vy / sqrt2) + sigmas[12];  // accel X
    measurement_sigmas[2] = (-acc_vx / sqrt2 + acc_vy / sqrt2) + sigmas[13]; // accel Y
    measurement_sigmas[3] = acc_vz + sigmas[14]; // accel Z

    // clip accel X and accel Y
    double acc_clip = 19.2882;
    if (fabs(measurement_sigmas[1]) > acc_clip) {
        measurement_sigmas[1] = (measurement_sigmas[1] > 0.0) ? acc_clip : -acc_clip;
    }
    double acc_y_clip = 19.6925;
    if (fabs(measurement_sigmas[2]) > acc_y_clip) {
        measurement_sigmas[2] = (measurement_sigmas[2] > 0.0) ? acc_y_clip : -acc_y_clip;
    }

    // vehicle frame gyro in degrees per second (no global gyro because it is defined in the state
    // vector as vehicle-frame, not global-frame like acceleration)
    double vehicle_gyro[3];
    for (int i = 0; i < 3; ++i) {
        vehicle_gyro[i] = sigmas[9 + i] * (180.0 / pi);
    }
    // 45-degree rotation and add biases
    double gyro_gx = vehicle_gyro[0];
    double gyro_gy = vehicle_gyro[1];
    double gyro_gz = vehicle_gyro[2];
    measurement_sigmas[4] = (gyro_gx / sqrt2 + gyro_gy / sqrt2) + sigmas[15]; // gyro x
    measurement_sigmas[5] = (-gyro_gx / sqrt2 + gyro_gy / sqrt2) + sigmas[16]; // gyro y
    measurement_sigmas[6] = gyro_gz + sigmas[17]; // gyro z

    // Magnetometer
    double mag_world_q[4] = {
        0.0,
        ukfh->mag_world[0],
        ukfh->mag_world[1],
        ukfh->mag_world[2]
    };

    // Rotate mag_world to vehicle frame: conj * mag_world_q * quat
    double temp_mag[4];
    quaternion_product_f64(quat_state_conj, mag_world_q, temp_mag);
    double mag_vehicle[4];
    quaternion_product_f64(temp_mag, quat_state, mag_vehicle);

    // Apply R_vehicle_to_mag: [x, y, -z]
    double mag_vx = mag_vehicle[1];
    double mag_vy = mag_vehicle[2];
    double mag_vz = mag_vehicle[3];
    measurement_sigmas[7] = mag_vx;  // mag_sensor x
    measurement_sigmas[8] = mag_vy;  // y
    measurement_sigmas[9] = -mag_vz; // z (flipped)
}

void calculate_initial_orientation(const double *imu_accel, const double *mag_field, double *init_quaternion, double *mag_world_frame) {
    
}