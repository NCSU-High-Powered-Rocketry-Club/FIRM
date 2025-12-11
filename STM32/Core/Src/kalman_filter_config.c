#include "kalman_filter_config.h"

//This function takes in altitude and returns a value for pressure. 
//Using data from https://www.engineeringtoolbox.com/standard-atmosphere-d_604.html
//Plotting the data point's, we found a relationship and that is what the function 
//Uses to calculate pressure. This is super doo doo, please don't murder us.
double altitude_To_Air_Density_Table(double altitude){  
    double pressure;
    //ad =-0.0807*altitude+1.2539
    pressure = -0.0807 * altitude + 1.2539;
    return pressure;
}

double ukf_initial_state_estimate[UKF_STATE_DIMENSION] = {
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 1.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0
};

double ukf_initial_state_covariance_diag[UKF_STATE_DIMENSION - 1] = {
    1e-6, 1e-6, 1e-6,
    1e-6, 1e-6, 1e-6,
    1e-2, 1e-2, 1e-2,
    1e-5, 1e-5, 1e-5,
    1e-4, 1e-4, 1e-4,
    1e-4, 1e-4, 1e-4,
    1e-1, 1e-1, 1e-1,
};

double ukf_state_process_covariance_diag[STATE_MACHINE_NUM_STATES][UKF_STATE_DIMENSION - 1] = {
    { // Standby
        1e-5, 1e-5, 1e-5, // position (x, y, z)
        1e-5, 1e-5, 1e-5, // velocity (x, y, z)
        1e-3, 1e-3, 1e-3, // acceleration (x, y, z)
        1, 1, 1, // gyro (x, y, z)
        0, 0, 0, // accelerometer offset (x, y, z)
        0, 0, 0, // gyroscope offset (x, y, z)
        1e-1, 1e-1, 1e-1 // orientation (r, p, y)
    },
    { // Motor Burn
        1e-2, 1e-2, 1e-2, // position (x, y, z)
        1e-2, 1e-2, 1e-2, // velocity (x, y, z)
        1e1, 1e1, 1e1, // acceleration (x, y, z)
        1e4, 1e4, 1e4, // gyro (x, y, z)
        0, 0, 0, // accelerometer offset (x, y, z)
        0, 0, 0, // gyroscope offset (x, y, z)
        1e2, 1e2, 1e2 // orientation (r, p, y)
    },
    { // Coast
        1e-2, 1e-2, 1e-2, // position (x, y, z)
        1e-3, 1e-3, 1e-3, // velocity (x, y, z)
        1e1, 1e1, 1e1, // acceleration (x, y, z)
        1e3, 1e3, 1e3, // gyro (x, y, z)
        0, 0, 0, // accelerometer offset (x, y, z)
        0, 0, 0, // gyroscope offset (x, y, z)
        1e1, 1e1, 1e1 // orientation (r, p, y)
    },
    { // Descent
        1e-1, 1e-1, 1e-1, // position (x, y, z)
        1, 1, 1, // velocity (x, y, z)
        1, 1, 1, // acceleration (x, y, z)
        1e2, 1e2, 1e2, // gyro (x, y, z)
        0, 0, 0, // accelerometer offset (x, y, z)
        0, 0, 0, // gyroscope offset (x, y, z)
        1e1, 1e1, 1e1 // orientation (r, p, y)
    },
    { // Landed
        1, 1, 1, // position (x, y, z)
        1, 1, 1, // velocity (x, y, z)
        1e2, 1e2, 1e2, // acceleration (x, y, z)
        1e2, 1e2, 1e2, // gyro (x, y, z)
        0, 0, 0, // accelerometer offset (x, y, z)
        0, 0, 0, // gyroscope offset (x, y, z)
        1, 1, 1 // orientation (r, p, y)
    }
};

double ukf_measurement_noise_covariance_diag[STATE_MACHINE_NUM_STATES][UKF_MEASUREMENT_DIMENSION] = {
    { // Standby
        1e2, // pressure (pascals)
        1e-2, 1e-2, 1e-2, // acceleration (x, y, z)
        1e-3, 1e-3, 1e-3, // gyro angular rate (x, y, z)
        1e-2, 1e-2, 1e-2 // normalized magnetometer (x, y, z)
    },
    { // Motor Burn
        1e3, // pressure (pascals)
        1e-2, 1e-2, 1e-2, // acceleration (x, y, z)
        1e2, 1e2, 1e2, // gyro angular rate (x, y, z)
        1e-3, 1e-3, 1e-3 // normalized magnetometer (x, y, z)
    },
    { // Coast
        1e3, // pressure (pascals)
        1e-1, 1e-1, 1e-1, // acceleration (x, y, z)
        1, 1, 1, // gyro angular rate (x, y, z)
        1e-3, 1e-3, 1e-3 // normalized magnetometer (x, y, z)
    },
    { // Descent
        1e3, // pressure (pascals)
        1e-1, 1e-1, 1e-1, // acceleration (x, y, z)
        1e2, 1e2, 1e2, // gyro angular rate (x, y, z)
        1e-1, 1e-1, 1e-1 // normalized magnetometer (x, y, z)
    },
    { // Landed
        1e1, // pressure (pascals)
        1e-2, 1e-2, 1e-2, // acceleration (x, y, z)
        1e1, 1e1, 1e1, // gyro angular rate (x, y, z)
        1e-1, 1e-1, 1e-1 // normalized magnetometer (x, y, z)
    }
};