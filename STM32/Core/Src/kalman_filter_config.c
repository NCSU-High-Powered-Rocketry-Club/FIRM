//This function takes in altitude and returns a value for pressure. 
//Using data from https://www.engineeringtoolbox.com/standard-atmosphere-d_604.html
//Plotting the data point's, we found a relationship and that is what the function 
//Uses to calculate pressure. This is super doo doo, please don't murder us.
float altitude_To_Air_Density_Table(float altitude){  
    float pressure;
    //ad =-0.0807*altitude+1.2539
    pressure = -0.0807*altitude + 1.2539;
    return pressure;
}