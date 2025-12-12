#include "state_machine.h"

State state = STANDBY;

/**
 * @brief sets the process and measurement noise covariance matrices when the state switches
 * @param ukfh ukf struct handle pointer
 */
static void set_state_matrices(struct UKF* ukfh) {
    for (int i = 0; i < UKF_STATE_DIMENSION - 1; i++) {
        ukfh->Q[i + (i * (UKF_STATE_DIMENSION - 1))] = ukf_state_process_covariance_diag[state][i];
    }
    for (int i = 0; i < UKF_MEASUREMENT_DIMENSION; i++) {
        ukfh->R[i + (i * (UKF_MEASUREMENT_DIMENSION - 1))] = ukf_measurement_noise_covariance_diag[state][i];
    }
}

void init_state(struct UKF* ukfh) {
    state = STANDBY;
    memset(ukfh->Q, 0, sizeof(double) * (UKF_STATE_DIMENSION - 1) * (UKF_STATE_DIMENSION - 1));
    memset(ukfh->R, 0, sizeof(double) * UKF_MEASUREMENT_DIMENSION * UKF_MEASUREMENT_DIMENSION);
    set_state_matrices(ukfh);
}

void state_update(struct UKF* ukfh) {
    switch (state) {
        case STANDBY: {
            double* errors = ukfh->measurement_errors;
            // the sum of errors in acceleration x, y, and z axis
            double accel_error_sum = fabs(errors[1]) + fabs(errors[2]) + fabs(errors[3]);
            if (accel_error_sum > 30) {
                state = MOTOR_BURN;
                set_state_matrices(ukfh);
            }
            break;
        }
        case MOTOR_BURN: {
            // check if imu has capped out and increase expected noise in accelerometer
            for (int i = 1; i < 4; i++) {
                if (fabs(ukfh->measurement_vector[i]) > 19)
                    ukfh->R[i * UKF_MEASUREMENT_DIMENSION + i] = ukf_measurement_noise_covariance_diag[state][i] * 1e3;
            }
            // increase noise in pressure to filter transonic effects based on how fast the rocket is going
            ukfh->R[0] = ukf_measurement_noise_covariance_diag[state][0] * fmax(ukfh->X[5], 1.0);
            
            if (ukfh->X[2] > 15 && ukfh->X[8] < -0.1) {
                state = COAST;
                set_state_matrices(ukfh);
            }
            break;
        }
        case COAST: {
            // increase noise in pressure based on velocity to filter out transonic effects or airbrakes
            ukfh->R[0] = ukf_measurement_noise_covariance_diag[state][0] * fmax(ukfh->X[5], 1.0);

            if (ukfh->X[5] < 0) {
                state = DESCENT;
                set_state_matrices(ukfh);
            }
            break;
        }
        case DESCENT: {
            // start lowering certainty in acceleration and gyro predictions the closer the rocket
            // gets to landing, because hitting the ground throws off expected values
            if (ukfh->X[2] < 100) {
                for (int i = 6; i < 12; i++) {
                    double variance_multipler = pow(101 - ukfh->X[2], 1.5);
                    ukfh->Q[i + i * (UKF_STATE_DIMENSION - 1)] = ukf_state_process_covariance_diag[state][i] * variance_multipler;
                }
            }
            break;

            if (ukfh->X[2] <= GROUND_ALTITUDE_METERS) {
                double* errors = ukfh->measurement_errors;
                // the sum of errors in acceleration x, y, and z axis
                double accel_error_sum = errors[1] + errors[2] + errors[3];
                if (accel_error_sum > 30) {
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