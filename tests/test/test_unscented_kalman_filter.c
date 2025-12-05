#include "unity.h"
#include "matrixhelper.h"
#include "kalman_filter_config.h"
#include "unscented_kalman_filter.h"
#include "ukf_functions.h"
#include <string.h>
#include <stdlib.h>




void setup(void) {}
void tearDown(void) {}

void test_Wm() {
    UKF ukf;
    int ret = ukf_init(&ukf);
    TEST_ASSERT_FALSE(ret); // false when no error
    // hand-calculated value for weight component:
    double exp_weight_component = 238.0952380953902;
    // hand-calculated value for first value of Wm:
    double exp_first_wm = -9999.0;
    // hand-calculated value for first value of Wc:
    double exp_first_wc = -9996.0001;

    double exp_wm[43] = {exp_first_wm};
    double exp_wc[43] = {exp_first_wc};
    exp_wm[0] = exp_first_wm;
    exp_wc[0] = exp_first_wc;
    for (int i = 1; i < 43; i++) {
        exp_wm[i] = exp_weight_component;
        exp_wc[i] = exp_weight_component;
    }

    double *wm = ukf_test_get_Wm();
    double *wc = ukf_test_get_Wc();
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, exp_wm, wm, 43);
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, exp_wc, wc, 43);
}

void test_initial_values() {
    UKF ukf;
    int ret = ukf_init(&ukf);
    TEST_ASSERT_FALSE(ret); // false when no error

    double exp_initial_x[22] = {
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 1.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0, 0.0
    };
    double exp_initial_p_diag[21] = {
        1e-6, 1e-6, 1e-6,
        1e-6, 1e-6, 1e-6,
        1e-2, 1e-2, 1e-2,
        1e-5, 1e-5, 1e-5,
        1e-4, 1e-4, 1e-4,
        1e-4, 1e-4, 1e-4,
        1e-1, 1e-1, 1e-1,
    };

    // initialize expected P matrix
    double exp_initial_p[21][21];
    for (int col = 0; col < 21; col++) {
        for (int row = 0; row < 21; row++) {
            // set value when on the diagonal index
            if (col == row) {
                exp_initial_p[row][col] = exp_initial_p_diag[row];
                continue;
            }
            // set to 0 if not on diagonal
            exp_initial_p[row][col] = 0;
            // initialize Q as all 1's
            ukf.Q[col * 21 + row] = 1;
        }
    }

    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, exp_initial_x, ukf.X, 22);
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, exp_initial_p, ukf.P, 21*21);
}

void test_predict_sigma_points(void) {
    UKF ukf;
    int ret = ukf_init(&ukf);
    ukf.state_transition_function = ukf_state_transition_function;
    TEST_ASSERT_FALSE(ret); // false when no error
    // initialize Q as all 1's
    for (int i = 0; i < 21 * 21; i++) {
        ukf.Q[i] = 1;
    }
    ukf.X[21] = 1;
    ukf.X[20] = -0.5;
    ret = ukf_predict(&ukf, 1);
    TEST_ASSERT_FALSE(ret);
    
    double sigma_points[43][22];
    ukf_test_get_sigma_points(sigma_points);
    TEST_ASSERT_DOUBLE_WITHIN(1e-7, 4.58257799e-02, sigma_points[1][0]); // first value of second row (First positive chol)
    TEST_ASSERT_DOUBLE_WITHIN(1e-7, -4.58257340e-02, sigma_points[22][4]); // 5th value of 23rd row (first negative chol)
    TEST_ASSERT_DOUBLE_WITHIN(1e-7, 1.87082791e-05, sigma_points[3][15]); // 16th value of 4th row

    // test quaternion rows (pos and neg)
    double *quat_row_2 = &sigma_points[1][18];
    double *quat_row_23 = &sigma_points[22][18];
    double quat_row_2_exp[4] = {0.658506118, -0.00763561762, -0.302528398, 0.689048589};
    double quat_row_23_exp[4] = {0.673777354, 0.00763561762, -0.363613339, 0.643234883};
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, quat_row_2_exp, quat_row_2, 4);
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, quat_row_23_exp, quat_row_23, 4);
}

void test_predict_sigmas_f(void) {
    UKF ukf;
    int ret = ukf_init(&ukf);
    ukf.state_transition_function = ukf_state_transition_function;
    TEST_ASSERT_FALSE(ret); // false when no error
    ukf.X[2] = 4.0;
    ukf.X[4] = 1.0;
    ukf.X[6] = 30.0;
    ukf.X[7] = 2.0;
    ukf.X[8] = 2.5;
    ukf.X[9] = 6.4;
    ukf.X[10] = 0.2;
    ukf.X[11] = -0.3;
    ukf.X[12] = -0.6;
    ukf.X[18] = 4.0;
    ukf.X[19] = -2.0;
    ukf.X[20] = -0.1;
    ukf.X[21] = 0.5;
    ukf.flight_state = 3;
    for (int i = 0; i < 21*21; i++) {
        // initialize Q as all 10's
        ukf.Q[i] = 10;
    }


    ret = ukf_predict(&ukf, 0.1);
    TEST_ASSERT_EQUAL(0, ret);
    double *q_scaled = ukf_test_get_Q_scaled();
    TEST_ASSERT_FLOAT_WITHIN(1e-7, -20.9979, ukf_test_get_lambda());
    double exp_q_scaled[21] = {
        0.0021,     0.0021,     0.0021,     0.0021,     0.0021,     0.0021,      
        0.002121,   0.0021,     0.0021,     0.0021,     0.0021,     0.0021,      
        0.0021,     0.0021,     0.0021,     0.0021,     0.0021,     0.0021,      
        0.0021,     0.0021,     0.0021,
    };
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, exp_q_scaled, &q_scaled[21*6], 21);
    

    // sanity checking a sigma point, pre state transition function
    double sigma_points[43][22];
    ukf_test_get_sigma_points(sigma_points);
    TEST_ASSERT_DOUBLE_WITHIN(1e-7, 4.04582573e+00, sigma_points[1][2]);
    
    // testing a few different rows, post state transition function
    double *sigmas_f = ukf_test_get_sigmas_f();
    double exp_first_row[22] = {
        0.00000000e+00,  1.00000000e-01,  4.00000000e+00,  2.93940000e+01,
        2.95960000e+00,  1.46970000e+00,  3.00000000e+01,  2.00000000e+00,
        2.50000000e+00,  6.40000000e+00,  2.00000000e-01, -3.00000000e-01,
        -6.0000000e-01,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  9.85036090e-01, -1.42941184e-01,
        1.60397740e-02,  9.49444332e-02,
    };
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, exp_first_row, sigmas_f, 22);
    double exp_second_row[22] = {
        5.04083533e-02,  1.50408307e-01,  4.05040831e+00,  2.94847258e+01,
        3.05032579e+00,  1.56042579e+00,  3.00458257e+01,  2.04582573e+00,
        2.54582573e+00,  6.44582573e+00,  2.45825734e-01, -2.54174266e-01,
        -5.5417426e-01,  4.58257340e-02,  4.58257340e-02,  4.58257340e-02,
        4.58257340e-02,  4.58257340e-02,  9.86364569e-01, -1.22432606e-01,
        5.43536782e-02,  9.56079064e-02,
    };
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, exp_second_row, &sigmas_f[22], 22);
    double exp_20th_row[22] = {
        0.00000000e+00,  1.00000000e-01,  4.00000000e+00,  2.93940000e+01,
        2.95960000e+00,  1.46970000e+00,  3.00000000e+01,  2.00000000e+00,
        2.50000000e+00,  6.40000000e+00,  2.00000000e-01, -3.00000000e-01,
        -6.0000000e-01,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  9.86034855e-01, -1.35827125e-01,
        1.69804731e-02,  9.48573691e-02,
    };
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, exp_20th_row, &sigmas_f[22*19], 22);
    double exp_40th_row[22] = {
        0.00000000e+00,  1.00000000e-01,  4.00000000e+00,  2.93940000e+01,
        2.95960000e+00,  1.46970000e+00,  3.00000000e+01,  2.00000000e+00,
        2.50000000e+00,  6.40000000e+00,  2.00000000e-01, -3.00000000e-01,
        -6.0000000e-01,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -4.58618243e-04,  9.85036057e-01, -1.42941473e-01,
        1.60392217e-02,  9.49444365e-02,
    };
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, exp_40th_row, &sigmas_f[22*39], 22);

    
}

void test_predict_unscented_transform(void) {
    UKF ukf;
    int ret = ukf_init(&ukf);
    ukf.state_transition_function = ukf_state_transition_function;
    TEST_ASSERT_EQUAL(0, ret); // false when no error
    ukf.X[2] = 4.0;
    ukf.X[4] = 1.0;
    ukf.X[6] = 30.0;
    ukf.X[7] = 2.0;
    ukf.X[8] = 2.5;
    ukf.X[9] = 6.4;
    ukf.X[10] = 0.2;
    ukf.X[11] = -0.3;
    ukf.X[12] = -0.6;
    ukf.X[18] = 4.0;
    ukf.X[19] = -2.0;
    ukf.X[20] = -0.1;
    ukf.X[21] = 0.5;
    ukf.flight_state = 3;
    for (int i = 0; i < 21*21; i++) {
        // initialize Q as all 10's
        ukf.Q[i] = 10;
    }


    ret = ukf_predict(&ukf, 0.1);
    TEST_ASSERT_EQUAL(0, ret);

    double *Wm = ukf_test_get_Wm();
    double exp_Wm[4] = {-9999.00000001, 238.0952381, 238.0952381, 238.0952381};
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, exp_Wm, Wm, 4);

    double *last_weighted_vector = ukf_test_get_weighted_vector_sigmas();
    double exp_last_weighted_vector[18] = {
        0.0,          23.80952381,  952.38095238, 6998.57142858,  704.66666667,
        349.92857143, 7142.85714286,  476.19047619,  595.23809524, 1523.80952381,
        47.61904762,  -71.42857143, -142.85714286,    0.0,           0.0,
        0.0,           0.0,          0.0,  
    };
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, exp_last_weighted_vector, last_weighted_vector, 18);

    double exp_X[22] = {
        -6.5702651e-16,  1.00000000e-01,  4.00000000e+00,  2.93940000e+01,
        2.95960000e+00,  1.46970000e+00,  3.00000000e+01,  2.00000000e+00,
        2.50000000e+00,  6.40000000e+00,  2.00000000e-01, -3.00000000e-01,
        -6.0000000e-01,  1.58206781e-15,  2.22044605e-16,  9.71445147e-16,
        -2.5812685e-15, -3.15025783e-15,  9.73778145e-01, -2.00354183e-01,
        4.35728055e-02,  9.85684307e-02,
    };
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, exp_X, ukf.X, 22);
    double exp_P_first_row[21] = {
        1.21000101,  1.21,        1.21,        2.1777801,   2.17778,     2.17778,
        1.1,         1.1,         1.1,         1.1,       1.1,         1.1,
        1.1,         1.1,         1.1,         1.1,       1.1,         1.1,
        0.76287306,  1.8024096,  -0.79415629,
    };
    double exp_P_last_row[21] = {
        -0.79415629, -0.79415629, -0.79415629, -1.42933693, -1.42933693, -1.42933693,
        -0.72196027, -0.72196027, -0.72196027, -0.72196032, -0.72196106, -0.72195966,
        -0.72196027, -0.72196027, -0.72196027, -0.72196027, -0.72196027, -0.72196027,
        -0.68221574, -1.22450985,  0.64794935,
    };
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-7, exp_P_last_row, &ukf.P[21*20], 21);
}

void test_predict_multiple_iterations(void) {
    UKF ukf;
    int ret = ukf_init(&ukf);
    ukf.state_transition_function = ukf_state_transition_function;
    TEST_ASSERT_EQUAL(0, ret); // false when no error
    ukf.X[2] = 4.0;
    ukf.X[4] = 1.0;
    ukf.X[6] = 30.0;
    ukf.X[7] = 2.0;
    ukf.X[8] = 2.5;
    ukf.X[9] = 6.4;
    ukf.X[10] = 0.2;
    ukf.X[11] = -0.3;
    ukf.X[12] = -0.6;
    ukf.X[18] = 4.0;
    ukf.X[19] = -2.0;
    ukf.X[20] = -0.1;
    ukf.X[21] = 0.5;
    ukf.flight_state = 3;
    for (int i = 0; i < 21*21; i++) {
        // initialize Q as all 10's
        ukf.Q[i] = 1;
    }

    
    double exp_X[5][22] = {
        {
            1.49186219e-16,  1.00000000e-01,  4.00000000e+00,  2.93940000e+01,
            2.95960000e+00,  1.46970000e+00,  3.00000000e+01,  2.00000000e+00,
            2.50000000e+00,  6.40000000e+00,  2.00000000e-01, -3.00000000e-01,
            -6.0000000e-01, -1.38777878e-16, -3.05311332e-16, -3.19189120e-16,
            4.16333634e-16, -1.38777878e-17,  9.83341585e-01, -1.53492113e-01,
            1.82000677e-02,  9.56465122e-02,
        },
        {
            2.93940000e+00,  3.95960000e-01,  4.14697000e+00,  5.87880000e+01,
            4.91920000e+00,  2.93940000e+00,  3.00000000e+01,  2.00000000e+00,
            2.50000000e+00,  6.40000000e+00,  2.00000000e-01, -3.00000000e-01,
            -6.0000000e-01,  8.78926561e-16, -7.39487836e-16,  2.83503379e-16,
            4.07213945e-15, -2.67643051e-16,  9.85393308e-01,  1.47069363e-01,
            5.93868548e-02,  6.19986454e-02,
        },
        {
            8.81820000e+00,  8.87880000e-01,  4.44091000e+00,  8.81820000e+01,
            6.87880000e+00,  4.40910000e+00,  3.00000000e+01,  2.00000000e+00,
            2.50000000e+00,  6.40000000e+00,  2.00000000e-01, -3.00000000e-01,
            -6.0000000e-01,  6.87721239e-16, -3.91693580e-16, -7.79421982e-16,
            5.28441638e-15, -1.28383699e-15,  9.00557437e-01,  4.22561210e-01,
            1.01673903e-01,  1.00371170e-02,
        },
        {
            1.76364000e+01,  1.57576000e+00,  4.88182000e+00,  1.17576000e+02,
            8.83840000e+00,  5.87880000e+00,  3.00000000e+01,  2.00000000e+00,
            2.50000000e+00,  6.40000000e+00,  2.00000000e-01, -3.00000000e-01,
            -6.0000000e-01, -6.30563217e-16, -9.62493867e-16, -2.53440911e-15,
            8.51872125e-15,  7.33102428e-17,  7.48288246e-01,  6.43120173e-01,
            1.55796895e-01, -4.67811082e-02,
        },
        {
            2.93940000e+01,  2.45960000e+00,  5.46970000e+00,  1.46970000e+02,
            1.07980000e+01,  7.34850000e+00,  3.00000000e+01,  2.00000000e+00,
            2.50000000e+00,  6.40000000e+00,  2.00000000e-01, -3.00000000e-01,
            -6.0000000e-01, -7.23224431e-16, -1.75249056e-15,  7.90730775e-17,
            1.17010618e-14,  1.08603319e-15,  5.59719613e-01,  7.96264914e-01,
            2.19195174e-01, -6.80412926e-02
        }
    };

    // do 5 predict iterations
    for (int i = 0; i < 5; i++) {
        char message[15];
        strcpy(message, "iteration: ");
        char num[2];
        sprintf(num, "%d", i);
        strcat(message, num);

        ret = ukf_predict(&ukf, 0.1);
        TEST_ASSERT_EQUAL_MESSAGE(0, ret, message);
        TEST_ASSERT_DOUBLE_ARRAY_WITHIN_MESSAGE(1e-7, exp_X[i], ukf.X, 22, message);
    }
}