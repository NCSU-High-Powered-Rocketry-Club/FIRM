#include "state_machine.h"

State state = STANDBY;

/**
 * @brief sets the process and measurement noise covariance matrices when the state switches
 * @param ukfh ukf struct handle pointer
 */
static void set_state_matrices(struct UKF* ukfh) {
    for (int i = 0; i < UKF_COVARIANCE_DIMENSION; i++) {
        ukfh->Q[i + (i * (UKF_COVARIANCE_DIMENSION))] = ukf_state_process_covariance_diag[state][i];
    }
    for (int i = 0; i < UKF_MEASUREMENT_DIMENSION; i++) {
        ukfh->R[i + (i * (UKF_MEASUREMENT_DIMENSION))] = ukf_measurement_noise_covariance_diag[state][i];
    }
}

void init_state(struct UKF* ukfh) {
    state = MOTOR_BURN;
    memset(ukfh->Q, 0, sizeof(float) * (UKF_COVARIANCE_DIMENSION) * (UKF_COVARIANCE_DIMENSION));
    memset(ukfh->R, 0, sizeof(float) * UKF_MEASUREMENT_DIMENSION * UKF_MEASUREMENT_DIMENSION);
    set_state_matrices(ukfh);
}

void state_update(struct UKF* ukfh) {
    switch (state) {
        case STANDBY: {
            float* errors = ukfh->measurement_errors;
            // the sum of errors in acceleration x, y, and z axis
            float accel_error_sum = fabsf(errors[1]) + fabsf(errors[2]) + fabsf(errors[3]);
            if (accel_error_sum > 30 && 0) {
                state = MOTOR_BURN;
                set_state_matrices(ukfh);
            }
            break;
        }
        case MOTOR_BURN: {
            // check if imu has capped out and increase expected noise in accelerometer
            for (int i = 1; i < 4; i++) {
                if (fabsf(ukfh->measurement_vector[i]) > 19.0F)
                    ukfh->R[i * UKF_MEASUREMENT_DIMENSION + i] = ukf_measurement_noise_covariance_diag[state][i] * 1e3F;
            }
            // increase noise in pressure to filter transonic effects based on how fast the rocket is going
            ukfh->R[0] = ukf_measurement_noise_covariance_diag[state][0] * fmaxf(ukfh->X[5], 1.0F);
            
            if (ukfh->X[2] > 15.0F && ukfh->X[8] < -0.1F && 0) {
                state = COAST;
                set_state_matrices(ukfh);
            }
            break;
        }
        case COAST: {
            // increase noise in pressure based on velocity to filter out transonic effects or airbrakes
            ukfh->R[0] = ukf_measurement_noise_covariance_diag[state][0] * fmaxf(ukfh->X[5], 1.0F);

            if (ukfh->X[5] < 0.0F) {
                state = DESCENT;
                set_state_matrices(ukfh);
            }
            break;
        }
        case DESCENT: {
            // start lowering certainty in acceleration and gyro predictions the closer the rocket
            // gets to landing, because hitting the ground throws off expected values
            if (ukfh->X[2] < 100.0F) {
                for (int i = 6; i < 12; i++) {
                    float variance_multipler = powf(101.0F - ukfh->X[2], 1.5F);
                    ukfh->Q[i + i * (UKF_COVARIANCE_DIMENSION)] = ukf_state_process_covariance_diag[state][i] * variance_multipler;
                }
            }
            break;

            if (ukfh->X[2] <= GROUND_ALTITUDE_METERS) {
                float* errors = ukfh->measurement_errors;
                // the sum of errors in acceleration x, y, and z axis
                float accel_error_sum = errors[1] + errors[2] + errors[3];
                if (accel_error_sum > 30.0F) {
                    state = LANDED;
                    set_state_matrices(ukfh);
                }
            }
            break;
        }
        case LANDED: {
            break;
        }
        default:
            break;
    }   
}