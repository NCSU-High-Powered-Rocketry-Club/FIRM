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