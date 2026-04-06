#pragma once
#include <float.h>
#include <math.h>
#include <stdint.h>

/**
 * @brief Instance structure for the floating-point matrix structure.
 */
typedef struct {
  uint16_t numRows; /**< number of rows of the matrix.     */
  uint16_t numCols; /**< number of columns of the matrix.  */
  float *pData;     /**< points to the data of the matrix. */
} matrix_instance_f32;

int symmetrize(matrix_instance_f32 *enter_matrix);

void rotvec_to_quat(const float rotvec[3], float quat[4]);
void quaternion_normalize_f32(float *quat);
void quaternion_product_f32(const float *q1, const float *q2, float *out);

void mat_add_f32(const matrix_instance_f32 *pSrcA, const matrix_instance_f32 *pSrcB,
                 matrix_instance_f32 *pDst);
void mat_sub_f32(const matrix_instance_f32 *pSrcA, const matrix_instance_f32 *pSrcB,
                 matrix_instance_f32 *pDst);
void mat_mult_f32(const matrix_instance_f32 *pSrcA, const matrix_instance_f32 *pSrcB,
                  matrix_instance_f32 *pDst);
void mat_trans_f32(const matrix_instance_f32 *pSrc, matrix_instance_f32 *pDst);

void mat_vec_mult_f32(const matrix_instance_f32 *pSrcA, const float *pVec, float *pDst);

void vec_mult_f32(const float *pSrcA, const float *pSrcB, float *pDst, int length);

void mat_inverse_f32(const matrix_instance_f32 *pSrc, matrix_instance_f32 *pDst);
