#pragma once
#include <arm_math.h>
#include <math.h>

int symmetrize(arm_matrix_instance_f64 *enter_matrix);

void rotvec_to_quat(const double rotvec[3], double quat[4]);

void quat_to_rotvec(const double quat[4], double rotvec[3]);

void mat_scale_f64(const arm_matrix_instance_f64 *pSrc, double scale, arm_matrix_instance_f64 *pDst);
void mat_add_f64(const arm_matrix_instance_f64 *pSrcA, const arm_matrix_instance_f64 *pSrcB, arm_matrix_instance_f64 *pDst);
void mat_vec_mult_f64(const arm_matrix_instance_f64 *pSrcA, const double *pVec, double *pDst);
void quaternion_normalize_f64(double *quat);
void quaternion_product_f64(const double *q1, const double *q2, double *out);
