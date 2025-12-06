#include "unity.h"
#include "matrixhelper.h"

void setUp(void) {}
void tearDown(void) {}

void test_symmetrize_2x2(void) {
    arm_matrix_instance_f64 m;
    double data[4] = {1.0, 2.0, 3.0, 4.0};
    m.numRows = 2;
    m.numCols = 2;
    m.pData = data;

    int ret = symmetrize(&m);
    TEST_ASSERT_FALSE(ret); // test for return value of 0
    double expected[4] = {1.0, 2.5, 2.5, 4.0};
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-6, expected, m.pData, 4);
}

void test_symmetrize_bad_size(void) {
    arm_matrix_instance_f64 m;
    double data[6] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    m.numRows = 2;
    m.numCols = 3;
    m.pData = data;

    int ret = symmetrize(&m);
    TEST_ASSERT(ret); // test for return value of 1
    // array value should not be modified.
    double expected[6] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-6, expected, m.pData, 6);
}

void test_symmetrize_negative_vals(void) {
    arm_matrix_instance_f64 m;
    double data[9] = {-1.7, 4.7, -6.2, -8.4, -8.0, 6.6, 1.3, -4.2, 1.8};
    m.numRows = 3;
    m.numCols = 3;
    m.pData = data;

    int ret = symmetrize(&m);
    TEST_ASSERT_FALSE(ret);
    double expected[9] = {-1.7, -1.85, -2.45, -1.85, -8.0, 1.2, -2.45, 1.2, 1.8};
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-6, expected, m.pData, 9);
}

void test_rotvec_to_quat_no_rotation(void) {
    double rotvec[3] = {0.0, 0.0, 0.0};
    double quat[4];
    rotvec_to_quat(rotvec, quat);

    double exp[4] = {1.0, 0.0, 0.0, 0.0};
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-6, exp, quat, 4);
}

void test_rotvec_to_quat(void) {
    double rotvec[3] = {2.0, 4.0, 6.0};
    double quat[4];
    rotvec_to_quat(rotvec, quat);
    double exp[4] = {-0.82529906, -0.15092133, -0.30184265, -0.45276398};
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-6, exp, quat, 4);

    double rotvec2[3] = {-0.4, -0.1, -3.9};
    double quat2[4];
    rotvec_to_quat(rotvec2, quat2);
    double exp2[4] = {-0.38025392, -0.09433399, -0.0235835, -0.91975642};
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-5, exp2, quat2, 4);

    double rotvec3[3] = {0.64, 0.02, -0.03};
    double quat3[4];
    rotvec_to_quat(rotvec3, quat3);
    double exp3[4] = {0.94907568160697, 0.314549404407839, 0.00982966888774497, -0.0147445033316174};
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-5, exp3, quat3, 4);
}

void test_quat_to_rotvec(void) {
    double quat[4] = {0.949, 0.315, 0.0098, -0.0147};
    double rotvec[3];
    quat_to_rotvec(quat, rotvec);
    double exp[3] = {0.64090345, 0.01993922, -0.02990883};
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-5, exp, rotvec, 3);

    double quat2[4] = {-0.438, -1.89345, 0.98543, 4.389};
    double rotvec2[3];
    quat_to_rotvec(quat2, rotvec2);
    double exp2[3] = {-1.28826116, 0.6704646, 2.98617773};
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-4, exp2, rotvec2, 3);
}

void test_backwards_compat_quat_rotvec(void) {
    // test that we can turn a rotation vector into a quaternion, then back into rotvec
    // and get the same value back
    double rotvec[3] = {0.834, -0.372, -1.247};
    double quat[4];
    rotvec_to_quat(rotvec, quat);
    double exp[3];
    memcpy(exp, rotvec, 3 * sizeof(double));
    quat_to_rotvec(quat, rotvec);
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-5, exp, rotvec, 3);
}

void test_mat_vec_mult_f64(void) {
    // 3x3 matrix times 3 element vector
    double mat_data[9] = {-4.5, 8.7, -3.3, 6.2, -1.7, 3.7, 1.2, -1.4, -6.6};
    arm_matrix_instance_f64 mat = {3, 3, mat_data};
    double vec[3] = {-2.5, 6.6, 2.3};
    double result[3];

    // confirmed with a matrix multiply calculator online
    double exp[3] = {61.08, -18.21, -27.42};
    mat_vec_mult_f64(&mat, vec, result);
    TEST_ASSERT_DOUBLE_ARRAY_WITHIN(1e-9, exp, result, 3);

}