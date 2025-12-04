#pragma once
#include <arm_math.h>
#include <math.h>

int symmetric(arm_matrix_instance_f64 *enter_matrix);

void rotvec_to_quat(const double rotvec[3], double quat[4]);

void quat_to_rotvec(const double quat[4], double rotvec[3]);

// Wrapper declarations for CMSIS-DSP f64 functions (not available in standard CMSIS)
void arm_mat_scale_f64_wrapper(const arm_matrix_instance_f64 *pSrc, double scale, arm_matrix_instance_f64 *pDst);
void arm_mat_add_f64_wrapper(const arm_matrix_instance_f64 *pSrcA, const arm_matrix_instance_f64 *pSrcB, arm_matrix_instance_f64 *pDst);
int arm_mat_cholesky_f64_wrapper(arm_matrix_instance_f64 *pSrc, arm_matrix_instance_f64 *pDst);
void arm_scale_f64_wrapper(const double *pSrc, double scale, double *pDst, uint32_t blockSize);
void arm_sub_f64_wrapper(const double *pSrcA, const double *pSrcB, double *pDst, uint32_t blockSize);
void arm_mult_f64_wrapper(const double *pSrcA, const double *pSrcB, double *pDst, uint32_t blockSize);
void arm_quaternion_norm_f64_wrapper(const double *pInputQ, double *pNorm, uint32_t numQ);
void arm_quaternion_product_single_f64_wrapper(const double *q1, const double *q2, double *out);

// Macro redirects to use the wrapper functions
#define arm_mat_scale_f64 arm_mat_scale_f64_wrapper
#define arm_mat_add_f64 arm_mat_add_f64_wrapper
#define arm_sub_f64 arm_sub_f64_wrapper
#define arm_mult_f64 arm_mult_f64_wrapper
#define arm_quaternion_norm_f64 arm_quaternion_norm_f64_wrapper
#define arm_quaternion_product_single_f64 arm_quaternion_product_single_f64_wrapper