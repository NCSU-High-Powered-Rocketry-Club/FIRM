#pragma once
#include <arm_math.h>
#include <math.h>

int symmetrize(arm_matrix_instance_f32 *enter_matrix);

void rotvec_to_quat(const float rotvec[3], float quat[4]);

void quat_to_rotvec(const float quat[4], float rotvec[3]);

void mat_scale_f32(const arm_matrix_instance_f32 *pSrc, float scale, arm_matrix_instance_f32 *pDst);
void mat_add_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst);
void mat_vec_mult_f32(const arm_matrix_instance_f32 *pSrcA, const float *pVec, float *pDst);
void quaternion_normalize_f32(float *quat);
void quaternion_product_f32(const float *q1, const float *q2, float *out);
