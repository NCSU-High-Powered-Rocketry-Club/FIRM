#include "unity.h"
#include "matrixhelper.h"
#include "kalman_filter_config.h"
#include "unscented_kalman_filter.h"
#include <string.h>

void setup(void) {}
void tearDown(void) {}

void test_Wm() {
    UKF ukf;
    int ret = ukf_init(&ukf);
    TEST_ASSERT_FALSE(ret); // false when no error
    // hand-calculated value for weight component:
    float exp_weight_component = 238.0952380953902F;
    // hand-calculated value for first value of Wm:
    float exp_first_wm = -9999.0F;
    // hand-calculated value for first value of Wc:
    float exp_first_wc = -9996.0001F;

    float exp_wm[43] = {exp_first_wm};
    float exp_wc[43] = {exp_first_wc};
    exp_wm[0] = exp_first_wm;
    exp_wc[0] = exp_first_wc;
    for (int i = 1; i < 43; i++) {
        exp_wm[i] = exp_weight_component;
        exp_wc[i] = exp_weight_component;
    }

    float *wm = ukf_test_get_Wm();
    float *wc = ukf_test_get_Wc();
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, exp_wm, wm, 43);
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, exp_wc, wc, 43);
}

void test_initial_values() {
    UKF ukf;
    int ret = ukf_init(&ukf);
    TEST_ASSERT_FALSE(ret); // false when no error

    float exp_initial_x[22] = {
        0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 1.0F,
        0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F,
        1.0F, 0.0F, 0.0F, 0.0F
    };
    float exp_initial_p_diag[21] = {
        1e-6F, 1e-6F, 1e-6F,
        1e-6F, 1e-6F, 1e-6F,
        1e-2F, 1e-2F, 1e-2F,
        1e-5F, 1e-5F, 1e-5F,
        1e-4F, 1e-4F, 1e-4F,
        1e-4F, 1e-4F, 1e-4F,
        1e-1F, 1e-1F, 1e-1F,
    };

    // initialize expected P matrix
    float exp_initial_p[21][21];
    for (int col = 0; col < 21; col++) {
        for (int row = 0; row < 21; row++) {
            // set value when on the diagonal index
            if (col == row) {
                exp_initial_p[row][col] = exp_initial_p_diag[row];
                continue;
            }
            // set to 0 if not on diagonal
            exp_initial_p[row][col] = 0;
        }
    }

    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, exp_initial_x, ukf.X, 22);
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, exp_initial_p, ukf.P, 21*21);
}

void test_predict(void) {
    UKF ukf;
    int ret = ukf_init(&ukf);
    TEST_ASSERT_FALSE(ret); // false when no error

    ukf.X[21] = 1;
    ukf.X[20] = -0.5F;
    ret = ukf_predict(&ukf, 1e-3);
    TEST_ASSERT_FALSE(ret);
    
    float *sigmas_f = ukf_test_get_sigmas_f();
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 4.58257799e-02, sigmas_f[22]); // first value of second row (First positive chol)
    TEST_ASSERT_FLOAT_WITHIN(1e-5, -4.58257340e-02, sigmas_f[22 * 22 + 4]); // 5th value of 23rd row (first negative chol)
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.87082791e-05, sigmas_f[3 * 22 + 15]); // 16th value of 4th row

    // test quaternion rows (pos and neg)
    float *quat_row_2 = &sigmas_f[22 * 1 + 18];
    float *quat_row_23 = &sigmas_f[22 * 22 + 18];
    float quat_row_2_exp[4] = {0.658506118F, -0.00763561762F, -0.302528398F, 0.689048589F};
    float quat_row_23_exp[4] = {0.673777354F, 0.00763561762F, -0.363613339F, 0.643234883F};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-5, quat_row_2_exp, quat_row_2, 4);
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-5, quat_row_23_exp, quat_row_23, 4);
}