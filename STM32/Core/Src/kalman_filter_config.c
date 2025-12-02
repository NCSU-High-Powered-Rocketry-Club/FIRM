#include "kalman_filter_config.h"

//This function takes in altitude and returns a value for pressure. 
//Using data from https://www.engineeringtoolbox.com/standard-atmosphere-d_604.html
//Plotting the data point's, we found a relationship and that is what the function 
//Uses to calculate pressure. This is super doo doo, please don't murder us.
float altitude_To_Air_Density_Table(float altitude){  
    float pressure;
    //ad =-0.0807*altitude+1.2539
    pressure = -0.0807F * altitude + 1.2539F;
    return pressure;
}

float ukf_initial_state_estimate[UKF_STATE_DIMENSION] = {
    0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F,
    0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F,
    1.0F, 0.0F, 0.0F, 0.0F
};

float ukf_initial_state_covariance_diag[UKF_STATE_DIMENSION - 1] = {
    1e-6F, 1e-6F, 1e-6F,
    1e-6F, 1e-6F, 1e-6F,
    1e-2F, 1e-2F, 1e-2F,
    1e-5F, 1e-5F, 1e-5F,
    1e-4F, 1e-4F, 1e-4F,
    1e-4F, 1e-4F, 1e-4F,
    1e-1F, 1e-1F, 1e-1F,
};