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
    float exp[4] = {-0.82529906F, -0.15092133F, -0.30184265F, -0.45276398F};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-7, exp, quat, 4);

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

void test_mat_sub_f32(void) {
    float a_data[6] = {1.0F, -2.0F, 3.5F, 0.25F, 8.0F, -0.5F};
    float b_data[6] = {0.5F, 1.0F, -1.5F, 0.25F, -2.0F, 2.5F};
    float out_data[6] = {0};

    arm_matrix_instance_f32 A = {2, 3, a_data};
    arm_matrix_instance_f32 B = {2, 3, b_data};
    arm_matrix_instance_f32 Out = {2, 3, out_data};

    mat_sub_f32(&A, &B, &Out);

    float exp[6] = {0.5F, -3.0F, 5.0F, 0.0F, 10.0F, -3.0F};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-7, exp, out_data, 6);

    // inputs must not be modified
    float exp_a[6] = {1.0F, -2.0F, 3.5F, 0.25F, 8.0F, -0.5F};
    float exp_b[6] = {0.5F, 1.0F, -1.5F, 0.25F, -2.0F, 2.5F};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-7, exp_a, a_data, 6);
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-7, exp_b, b_data, 6);
}

void test_vec_scale_f32_out_of_place(void) {
    float in[5] = {1.0F, -2.0F, 3.5F, 0.0F, -4.25F};
    float out[5] = {0};
    vec_scale_f32(in, 2.5F, out, 5);

    float exp[5] = {2.5F, -5.0F, 8.75F, 0.0F, -10.625F};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-7, exp, out, 5);
    // input must not be modified for out-of-place usage
    float exp_in[5] = {1.0F, -2.0F, 3.5F, 0.0F, -4.25F};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-7, exp_in, in, 5);
}

void test_vec_scale_f32_in_place_and_sign(void) {
    float data[4] = {2.0F, -1.0F, 0.25F, 8.0F};
    vec_scale_f32(data, -0.5F, data, 4);

    float exp[4] = {-1.0F, 0.5F, -0.125F, -4.0F};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-7, exp, data, 4);

    // scaling by 0 should yield all zeros
    vec_scale_f32(data, 0.0F, data, 4);
    float exp_zero[4] = {0.0F, 0.0F, 0.0F, 0.0F};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-7, exp_zero, data, 4);
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

void test_quaternion_normalize_f32(void) {
    float q_in[4] = {0.4F, -0.8F, -0.1F, 0.5F};
    float q_exp[4] = {0.38851434F, -0.7770287F, -0.09712858F, 0.4856429F};
    quaternion_normalize_f32(q_in);
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-7, q_exp, q_in, 4);
}

void test_quat_multiply_f32(void) {
    float q_in1[4] = {0.182574185835055F, 0.365148371670111F, 0.547722557505166F, 0.730296743340221F};
    float q_in2[4] = {0.599938784879706F, -0.799918379839608F, 0.0F, 0.0142842567828501F};
    float q_exp_in1[4] = {0.182574185835055F, 0.365148371670111F, 0.547722557505166F, 0.730296743340221F};
    float q_exp_in2[4] = {0.599938784879706F, -0.799918379839608F, 0.0F, 0.0142842567828501F};
    float exp_ans[4] = {0.39119048285816F, 0.0808460331240197F, -0.260793655238773F, 0.878874618154666F};
    float q_ans[4];
    quaternion_product_f32(q_in1, q_in2, q_ans);
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-7, exp_ans, q_ans, 4);
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-7, q_exp_in1, q_in1, 4);
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-7, q_exp_in2, q_in2, 4);
}

void test_quat_multiply_f32_overwrite(void) {
    float q_in1[4] = {0.182574185835055F, 0.365148371670111F, 0.547722557505166F, 0.730296743340221F};
    float q_in2[4] = {0.599938784879706F, -0.799918379839608F, 0.0F, 0.0142842567828501F};
    float exp_ans[4] = {0.39119048285816F, 0.0808460331240197F, -0.260793655238773F, 0.878874618154666F};
    quaternion_product_f32(q_in1, q_in2, q_in2);
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-7, exp_ans, q_in2, 4);
}