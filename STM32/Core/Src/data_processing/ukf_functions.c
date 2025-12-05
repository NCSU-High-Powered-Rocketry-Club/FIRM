#include "ukf_functions.h"
#include "dsp/quaternion_math_functions.h"

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