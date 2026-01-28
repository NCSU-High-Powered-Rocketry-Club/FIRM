#include "unity.h"
#include "matrixhelper.h"
#include "ukf_functions.h"
#include <stdlib.h>
#include <time.h>

#define DIM_X 16

void setUp(void) {}
void tearDown(void) {}

void test_state_transition_function_const_input(void) {
    // generate random seed based off time
    srand(time(NULL));

    float sigmas[DIM_X];
    float sigmas_copy[DIM_X];
    float prediction[DIM_X];
    // test for all 5 states
    for (int i = 0; i < 5; i++) {
        for (int sigma = 0; sigma < DIM_X; sigma++) {
            // random sigma between -10.0 and 10.0
            sigmas[sigma] = ((float)rand() / (float)RAND_MAX * 20.0) - 10.0;
        }
        memcpy(sigmas_copy, sigmas, DIM_X * sizeof(float));
        ukf_state_transition_function(sigmas, 0.1, i, prediction);
        TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, sigmas_copy, sigmas, DIM_X);
    }
    
}

void test_state_transition_function_standby(void) {
    float sigmas[DIM_X] = {
        4.58257799e-02F, 4.58257340e-02F, 4.58257340e-02F, 4.58257340e-02F,
        4.58257340e-02F, 4.58257340e-02F, 4.58257340e-02F, 4.58257340e-02F,
        1.04582573e+00F, 4.58257340e-02F, 4.58257340e-02F, 4.58257340e-02F,
        6.58506118e-01F, -7.63561762e-03F,-3.02528398e-01F, 6.89048589e-01F
    };

    float dt = 0.1;
    int state = 0;
    float prediction[DIM_X];
    float expected[DIM_X] = {
        0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000,
        0.04582573, 0.04582573, 1.04582572, 0.02291287, 0.02291287, 0.02291287,
        0.65850610, -0.00763562, -0.30252841, 0.68904859,
    };
    ukf_state_transition_function(sigmas, dt, state, prediction);
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, expected, prediction, DIM_X);
}

void test_state_transition_function_freefall(void) {
    // second row of sigmas
    float sigmas[DIM_X] = {
        1.44914492e-02F, 1.44913043e-02F, 4.01449130e+00F, 1.44913043e-02F,
        1.01449130e+00F, 1.44913043e-02F, 3.00144913e+01F, 2.01449130e+00F,
        2.51449130e+00F, 6.41449130e+00F, 2.14491304e-01F, -2.85508696e-01F,
        8.91175039e-01F, -4.38826757e-01F,-1.1751903e-02F, 1.14455320e-01F
    };

    float dt = 0.1;
    int state = 3;
    float prediction[DIM_X];
    float expected[DIM_X] = {
        0.01594058, 0.11594044, 4.01594019, 29.42269325, 2.98829007,
        1.49839008, 30.01449203, 2.01449132, 2.51449132, 6.41449118,
        0.21449131, -0.28550869, 0.98565638, -0.13649091, 0.02816648, 0.09517567
    };
    ukf_state_transition_function(sigmas, dt, state, prediction);
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, expected, prediction, DIM_X);
}

void test_measurement_function(void) {
    UKF ukf;
    ukf.initial_pressure = 101328.0F;
    ukf.mag_world[0] = 0.30942637F;
    ukf.mag_world[1] = 0.92827912F;
    ukf.mag_world[2] = -0.20628425F;

    float input_sigmas_f[DIM_X] = {
        2.939400000e+01F,  2.45960000e+00F,  5.46970000e+00F,  1.46970000e+02F,
        1.079801570e+01F,  7.34850786e+00F,  3.00000000e+01F,  2.00001604e+00F,
        2.500008020e+00F,  6.40000545e+00F,  2.00005452e-01F, -2.99994548e-01F,
        5.05660013e-01F,  8.43914113e-01F, 1.499743390e-01F, -9.81051397e-02F
    };
    float exp_measurement_sigmas[10] = {
        101262.30468750, 19.28820038, -11.20370770, -3.35659909, 267.39437866, -251.18824768, -17.18842316, 0.49787208, -0.47277495, 0.72705382
    };
    float measurement_sigmas[10];
    ukf_measurement_function(input_sigmas_f, &ukf, measurement_sigmas);
    // delta of 1e-6 doesnt work too well when the first element is 1e+5, so we will just scale
    // it down
    exp_measurement_sigmas[0] /= 1e5;
    measurement_sigmas[0] /= 1e5;
    // also 6th element
    exp_measurement_sigmas[5] /= 1e3;
    measurement_sigmas[5] /= 1e3;
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, exp_measurement_sigmas, measurement_sigmas, 10);
}
