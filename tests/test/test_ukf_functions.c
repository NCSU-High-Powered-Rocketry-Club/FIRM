#include <unity.h>
#include "matrixhelper.h"
#include "ukf_functions.h"
#include <stdlib.h>
#include <time.h>

void setup(void) {}
void tearDown(void) {}

void test_state_transition_function_const_input(void) {
    // generate random seed based off time
    srand(time(NULL));

    double sigmas[22];
    double sigmas_copy[22];
    double prediction[22];
    // test for all 5 states
    for (int i = 0; i < 5; i++) {
        for (int sigma = 0; sigma < 22; sigma++) {
            // random sigma between -10.0 and 10.0
            sigmas[sigma] = ((double)rand() / (double)RAND_MAX * 20.0) - 10.0;
        }
        memcpy(sigmas_copy, sigmas, 22 * sizeof(double));
        ukf_state_transition_function(sigmas, 0.1, i, prediction);
        TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, sigmas_copy, sigmas, 22);
    }
    
}

void test_state_transition_function_standby(void) {
    // second row of sigmas
    double sigmas[22] = {
        4.58257799e-02, 4.58257340e-02, 4.58257340e-02, 4.58257340e-02,
        4.58257340e-02, 4.58257340e-02, 4.58257340e-02, 4.58257340e-02,
        1.04582573e+00, 4.58257340e-02, 4.58257340e-02, 4.58257340e-02,
        4.58257340e-02, 4.58257340e-02, 4.58257340e-02, 4.58257340e-02,
        4.58257340e-02, 4.58257340e-02, 6.58506118e-01, -7.63561762e-03,
        -3.02528398e-01, 6.89048589e-01
    };

    double dt = 0.1;
    int state = 0;
    double prediction[22];
    double expected[22] = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.04582573,  0.04582573,  1.04582573,  0.02291287,  0.02291287,  0.02291287,
        0.04582573,  0.04582573,  0.04582573,  0.04582573,  0.04582573,  0.04582573,
        0.65850612, -0.00763562, -0.3025284,   0.68904859,
    };
    ukf_state_transition_function(sigmas, dt, state, prediction);
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, expected, prediction, 22);
}

void test_state_transition_function_freefall(void) {
    // second row of sigmas
    double sigmas[22] = {
        1.44914492e-02, 1.44913043e-02, 4.01449130e+00, 1.44913043e-02,
        1.01449130e+00, 1.44913043e-02, 3.00144913e+01, 2.01449130e+00,
        2.51449130e+00, 6.41449130e+00, 2.14491304e-01, -2.85508696e-01,
        -5.8550869e-01, 1.44913043e-02, 1.44913043e-02, 1.44913043e-02,
        1.44913043e-02, 1.44913043e-02, 8.91175039e-01, -4.38826757e-01,
        -1.1751903e-02, 1.14455320e-01
    };

    double dt = 0.1;
    int state = 3;
    double prediction[22];
    double expected[22] = {
        1.59405796e-02,  1.15940434e-01,  4.01594043e+00,  2.94226899e+01,
        2.98828988e+00,  1.49838988e+00,  3.00144913e+01,  2.01449130e+00,
        2.51449130e+00,  6.41449130e+00,  2.14491304e-01, -2.85508696e-01,
        -5.8550869e-01,  1.44913043e-02,  1.44913043e-02,  1.44913043e-02,
        1.44913043e-02,  1.44913043e-02,  9.85656367e-01, -1.36490909e-01,
        2.81664798e-02,  9.51756693e-02,
    };
    ukf_state_transition_function(sigmas, dt, state, prediction);
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, expected, prediction, 22);
}

void test_measurement_function(void) {
    UKF ukf;
    ukf.initial_pressure = 101328;
    ukf.mag_world[0] = 0.30942637;
    ukf.mag_world[1] = 0.92827912;
    ukf.mag_world[2] = -0.20628425;

    double input_sigmas_f[22] = {
        2.939400000e+01,  2.45960000e+00,  5.46970000e+00,  1.46970000e+02,
        1.079801570e+01,  7.34850786e+00,  3.00000000e+01,  2.00001604e+00,
        2.500008020e+00,  6.40000545e+00,  2.00005452e-01, -2.99994548e-01,
        -5.99994548e-01,  5.45181321e-06,  5.45181322e-06,  5.45181323e-06,
        5.451813200e-06,  5.45181320e-06,  5.05660013e-01,  8.43914113e-01,
        1.499743390e-01, -9.81051397e-02
    };
    double exp_measurement_sigmas[10] = {
        1.012623060e+05,  1.92882000e+01, -1.12037029e+01, -3.35659378e+00,   
        2.673943930e+02, -2.51188247e+02, -1.71884160e+01,  4.97872079e-01,
        -4.72774960e-01,  7.27053801e-01
    };
    double measurement_sigmas[10];
    ukf_measurement_function(input_sigmas_f, &ukf, measurement_sigmas);
    // delta of 1e-7 doesnt work too well when the first element is 1e+5, so we will just scale
    // it down
    exp_measurement_sigmas[0] /= 1e5;
    measurement_sigmas[0] /= 1e5;
    // also 6th element
    exp_measurement_sigmas[5] /= 1e3;
    measurement_sigmas[5] /= 1e3;
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, exp_measurement_sigmas, measurement_sigmas, 10);
}