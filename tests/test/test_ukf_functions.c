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