#pragma once
#include <stdint.h>
#include <math.h>

  /**
   * @brief Instance structure for the floating-point matrix structure.
   */
  typedef struct
  {
    uint16_t numRows;     /**< number of rows of the matrix.     */
    uint16_t numCols;     /**< number of columns of the matrix.  */
    float *pData;     /**< points to the data of the matrix. */
  } arm_matrix_instance_f32;

int symmetrize(arm_matrix_instance_f32 *enter_matrix);

void rotvec_to_quat(const float rotvec[3], float quat[4]);
void quat_to_rotvec(const float quat[4], float rotvec[3]);
void quaternion_normalize_f32(float *quat);
void quaternion_product_f32(const float *q1, const float *q2, float *out);

void mat_add_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst);
void mat_sub_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst);
void mat_scale_f32(const arm_matrix_instance_f32 *pSrc, float scale, arm_matrix_instance_f32 *pDst);
void mat_mult_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst);
void mat_trans_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst);

void mat_vec_mult_f32(const arm_matrix_instance_f32 *pSrcA, const float *pVec, float *pDst);

void vec_add_f32(const float *pSrcA, const float *pSrcB, float *pDst, int length);
void vec_sub_f32(const float *pSrcA, const float *pSrcB, float *pDst, int length);
void vec_scale_f32(const float *pSrcA, float scale, float *pDst, int length);
void vec_mult_f32(const float *pSrcA, const float *pSrcB, float *pDst, int length);

int mat_cholesky_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst);
void mat_inverse_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst);
