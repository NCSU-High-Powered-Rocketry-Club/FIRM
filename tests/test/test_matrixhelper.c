#include "unity.h"
#include "matrixhelper.h"

void setUp(void) {}
void tearDown(void) {}

void test_symmetrize_2x2(void) {
    arm_matrix_instance_f32 m;
    float data[4] = {1.0, 2.0, 3.0, 4.0};
    m.numRows = 2;
    m.numCols = 2;
    m.pData = data;

    int ret = symmetrize(&m);
    TEST_ASSERT_FALSE(ret); // test for return value of 0
    float expected[4] = {1.0, 2.5, 2.5, 4.0};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, expected, m.pData, 4);
}

void test_symmetrize_bad_size(void) {
    arm_matrix_instance_f32 m;
    float data[6] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    m.numRows = 2;
    m.numCols = 3;
    m.pData = data;

    int ret = symmetrize(&m);
    TEST_ASSERT(ret); // test for return value of 1
    // array value should not be modified.
    float expected[6] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, expected, m.pData, 6);
}

void test_symmetrize_negative_vals(void) {
    arm_matrix_instance_f32 m;
    float data[9] = {-1.7, 4.7, -6.2, -8.4, -8.0, 6.6, 1.3, -4.2, 1.8};
    m.numRows = 3;
    m.numCols = 3;
    m.pData = data;

    int ret = symmetrize(&m);
    TEST_ASSERT_FALSE(ret);
    float expected[9] = {-1.7, -1.85, -2.45, -1.85, -8.0, 1.2, -2.45, 1.2, 1.8};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, expected, m.pData, 9);
}

void test_rotvec_to_quat_no_rotation(void) {
    float rotvec[3] = {0.0, 0.0, 0.0};
    float quat[4];
    rotvec_to_quat(rotvec, quat);

    float exp[4] = {1.0, 0.0, 0.0, 0.0};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, exp, quat, 4);
}

void test_rotvec_to_quat(void) {
    float rotvec[3] = {2.0, 4.0, 6.0};
    float quat[4];
    rotvec_to_quat(rotvec, quat);
    float exp[4] = {-0.82529906, -0.15092133, -0.30184265, -0.45276398};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, exp, quat, 4);

    float rotvec2[3] = {-0.4, -0.1, -3.9};
    float quat2[4];
    rotvec_to_quat(rotvec2, quat2);
    float exp2[4] = {-0.38025392, -0.09433399, -0.0235835, -0.91975642};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-5, exp2, quat2, 4);

    float rotvec3[3] = {0.64, 0.02, -0.03};
    float quat3[4];
    rotvec_to_quat(rotvec3, quat3);
    float exp3[4] = {0.94907568160697, 0.314549404407839, 0.00982966888774497, -0.0147445033316174};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-5, exp3, quat3, 4);
}

void test_quat_to_rotvec(void) {
    float quat[4] = {0.949, 0.315, 0.0098, -0.0147};
    float rotvec[3];
    quat_to_rotvec(quat, rotvec);
    float exp[3] = {0.64090345, 0.01993922, -0.02990883};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-5, exp, rotvec, 3);

    float quat2[4] = {-0.438, -1.89345, 0.98543, 4.389};
    float rotvec2[3];
    quat_to_rotvec(quat2, rotvec2);
    float exp2[3] = {-1.28826116, 0.6704646, 2.98617773};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-4, exp2, rotvec2, 3);
}

void test_backwards_compat_quat_rotvec(void) {
    // test that we can turn a rotation vector into a quaternion, then back into rotvec
    // and get the same value back
    float rotvec[3] = {0.834, -0.372, -1.247};
    float quat[4];
    rotvec_to_quat(rotvec, quat);
    float exp[3];
    memcpy(exp, rotvec, 3 * sizeof(float));
    quat_to_rotvec(quat, rotvec);
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-5, exp, rotvec, 3);
}

void test_mat_vec_mult_f32(void) {
    // 3x3 matrix times 3 element vector
    float mat_data[9] = {-4.5, 8.7, -3.3, 6.2, -1.7, 3.7, 1.2, -1.4, -6.6};
    arm_matrix_instance_f32 mat = {3, 3, mat_data};
    float vec[3] = {-2.5, 6.6, 2.3};
    float result[3];

    // confirmed with a matrix multiply calculator online
    float exp[3] = {61.08, -18.21, -27.42};
    mat_vec_mult_f32(&mat, vec, result);
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-9, exp, result, 3);

}

void test_mat_inv_kalman_gain(void) {
    float pxy_data[20];
    for (int i = 1; i <= 20; i++) {
        pxy_data[i-1] = (float)i;
    };
    arm_matrix_instance_f32 pxy = {5, 4, pxy_data};
    float s_data[16] = {5.0F, 2.0F, 0.0F, 0.0F, 2.0F, 5.0F, 1.0F, 0.0F, 0.0F, 1.0F, 3.0F, 1.0F, 0.0F, 0.0F, 1.0F, 4.0F};
    arm_matrix_instance_f32 s = {4, 4, s_data};
    float result_inv_data[20];
    arm_matrix_instance_f32 result_inv = {5, 4, result_inv_data};
    float s_inv_data[16];
    arm_matrix_instance_f32 s_inv = {4, 4, s_inv_data};
    arm_mat_inverse_f32(&s, &s_inv);
    mat_mult_f32(&pxy, &s_inv, &result_inv);


    float s2_data[16] = {5.0F, 2.0F, 0.0F, 0.0F, 2.0F, 5.0F, 1.0F, 0.0F, 0.0F, 1.0F, 3.0F, 1.0F, 0.0F, 0.0F, 1.0F, 4.0F};
    arm_matrix_instance_f32 s2 = {4, 4, s2_data};
    float pxy2_data[20];
    for (int i = 1; i <= 20; i++) {
        pxy2_data[i-1] = (float)i;
    };
    arm_matrix_instance_f32 pxy2 = {5, 4, pxy2_data};
    float result_data[16];
    arm_matrix_instance_f32 result = {5, 4, result_data};
    int ret = get_kalman_gain(&pxy2, &s2, &result);
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-5, result_inv_data, result_data, 20);    
}

