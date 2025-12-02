#include "unity.h"
#include "matrixhelper.h"

void setup(void) {}
void tearDown(void) {}

void test_symmetric_2x2(void) {
    arm_matrix_instance_f32 m;
    float data[4] = {1.0, 2.0, 3.0, 4.0};
    m.numRows = 2;
    m.numCols = 2;
    m.pData = data;

    int ret = symmetric(&m);
    TEST_ASSERT_FALSE(ret); // test for return value of 0
    float expected[4] = {1.0, 2.5, 2.5, 4.0};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, expected, m.pData, 4);
}

void test_symmetric_bad_size(void) {
    arm_matrix_instance_f32 m;
    float data[6] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    m.numRows = 2;
    m.numCols = 3;
    m.pData = data;

    int ret = symmetric(&m);
    TEST_ASSERT(ret); // test for return value of 1
    // array value should not be modified.
    float expected[6] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, expected, m.pData, 6);
}

void test_symmetric_negative_vals(void) {
    arm_matrix_instance_f32 m;
    float data[9] = {-1.7, 4.7, -6.2, -8.4, -8.0, 6.6, 1.3, -4.2, 1.8};
    m.numRows = 3;
    m.numCols = 3;
    m.pData = data;

    int ret = symmetric(&m);
    TEST_ASSERT_FALSE(ret);
    float expected[9] = {-1.7, -1.85, -2.45, -1.85, -8.0, 1.2, -2.45, 1.2, 1.8};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, expected, m.pData, 9);
}

void test_rotvec_to_quat_no_rotation(void) {
    float rotvec[3] = {0.0F, 0.0F, 0.0F};
    float quat[4];
    rotvec_to_quat(rotvec, quat);

    float exp[4] = {1.0F, 0.0F, 0.0F, 0.0F};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, exp, quat, 4);
}

void test_rotvec_to_quat(void) {
    float rotvec[3] = {2.0F, 4.0F, 6.0F};
    float quat[4];
    rotvec_to_quat(rotvec, quat);
    float exp[4] = {-0.82529906, -0.15092133, -0.30184265, -0.45276398};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, exp, quat, 4);

    float rotvec2[3] = {-0.4F, -0.1F, -3.9F};
    float quat2[4];
    rotvec_to_quat(rotvec2, quat2);
    float exp2[4] = {-0.38025392, -0.09433399, -0.0235835, -0.91975642};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-5, exp2, quat2, 4);
}