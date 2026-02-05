#include "matrixhelper.h"

int symmetrize(arm_matrix_instance_f32 *enter_matrix) {
    uint16_t mat_rows = enter_matrix->numRows;
    uint16_t mat_cols = enter_matrix->numCols;
    if (mat_rows != mat_cols) {
        return 1;
    }
    
    for (int row = 0; row < mat_rows; row++) {
        for (int col = row + 1; col < mat_cols; col++) {
            float right_val = enter_matrix->pData[row * mat_cols + col];
            float left_val = enter_matrix->pData[col * mat_cols + row];
            float avg = 0.5F * (right_val + left_val);
            enter_matrix->pData[row * mat_cols + col] = avg;
            enter_matrix->pData[col * mat_cols + row] = avg;
        }
    }
    
    return 0;
}

void rotvec_to_quat(const float rotvec[3], float quat[4]) {
    float theta;
    theta = sqrtf(rotvec[0] * rotvec[0] + rotvec[1] * rotvec[1] + rotvec[2] * rotvec[2]);
    theta *= 0.5F;
    if (theta == 0.0) {
        quat[0] = 1.0F;
        quat[1] = 0.0F;
        quat[2] = 0.0F;
        quat[3] = 0.0F;
        return;
    }

    float s = sinf(theta) / theta;
    quat[0] = cosf(theta);
    quat[1] = (rotvec[0] * 0.5F) * s;
    quat[2] = (rotvec[1] * 0.5F) * s;
    quat[3] = (rotvec[2] * 0.5F) * s;
}

void quat_to_rotvec(const float quat[4], float rotvec[3]) {
    // Normalize quaternion manually (no arm_quaternion_norm_f32 available)
    float quat_norm = sqrtf(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);

    float w = quat[0] / quat_norm;
    float x = quat[1] / quat_norm;
    float y = quat[2] / quat_norm;
    float z = quat[3] / quat_norm;

    float half_angle = acosf(w);
    float theta = 2.0F * half_angle;
    float sin_half = sinf(half_angle);
    if (fabsf(sin_half) < 1e-6) {
        rotvec[0] = 0.0F;
        rotvec[1] = 0.0F;
        rotvec[2] = 0.0F;
        return;
    }

    float scale = theta / sin_half;
    rotvec[0] = x * scale;
    rotvec[1] = y * scale;
    rotvec[2] = z * scale;
}

// cmsis-dsp doesnt have f32 matrix scale
void mat_scale_f32(const arm_matrix_instance_f32 *pSrc, float scale, arm_matrix_instance_f32 *pDst) {
    int numElements = pSrc->numRows * pSrc->numCols;
    for (int i = 0; i < numElements; i++) {
        pDst->pData[i] = pSrc->pData[i] * scale;
    }
}

// cmsis-dsp doesnt have f32 matrix add
void mat_add_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst) {
    int numElements = pSrcA->numRows * pSrcA->numCols;
    for (int i = 0; i < numElements; i++) {
        pDst->pData[i] = pSrcA->pData[i] + pSrcB->pData[i];
    }
}

void mat_sub_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst) {
    int numElements = pSrcA->numRows * pSrcA->numCols;
    for (int i = 0; i < numElements; i++) {
        pDst->pData[i] = pSrcA->pData[i] - pSrcB->pData[i];
    }
}

// cmsis-dsp doesnt have f32 matrix/vector multiply
void mat_vec_mult_f32(const arm_matrix_instance_f32 *pSrcA, const float *pVec, float *pDst) {
    uint16_t rows = pSrcA->numRows;
    uint16_t cols = pSrcA->numCols;
    for (int i = 0; i < rows; i++) {
        pDst[i] = 0.0F;
        for (int j = 0; j < cols; j++) {
            pDst[i] += pSrcA->pData[i * cols + j] * pVec[j];
        }
    }
}

// Quaternion norm
void quaternion_normalize_f32(float *quat) {
    float norm = sqrtf(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
    quat[0] /= norm;
    quat[1] /= norm;
    quat[2] /= norm;
    quat[3] /= norm;
}

// out = q1 * q2 (Hamilton product)
void quaternion_product_f32(const float *q1, const float *q2, float *out) {
    float w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
    float w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];
    
    out[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    out[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    out[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    out[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2;
}

void mat_mult_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst) {
    int a_rows = pSrcA->numRows;
    int a_cols = pSrcA->numCols;
    int b_cols = pSrcB->numCols;

    for (int i = 0; i < a_rows; i++) {
        for (int j = 0; j < b_cols; j++) {
            float sum = 0.0F;
            for (int k = 0; k < a_cols; k++) {
                sum += pSrcA->pData[i * a_cols + k] * pSrcB->pData[k * b_cols + j];
            }
            pDst->pData[i * b_cols + j] = sum;
        }
    }
}

void vec_sub_f32(const float *pSrcA, const float *pSrcB, float *pDst, int length) {
    for (int i = 0; i < length; i++) {
        pDst[i] = pSrcA[i] - pSrcB[i];
    }
}

void vec_add_f32(const float *pSrcA, const float *pSrcB, float *pDst, int length) {
    for (int i = 0; i < length; i++) {
        pDst[i] = pSrcA[i] + pSrcB[i];
    }
}

void vec_scale_f32(const float *pSrcA, const float scale, float *pDst, int length) {
    for (int i = 0; i < length; i++) {
        pDst[i] = pSrcA[i] * scale;
    }
}

void vec_mult_f32(const float *pSrcA, const float *pSrcB, float *pDst, int length) {
    for (int i = 0; i < length; i++) {
        pDst[i] = pSrcA[i] * pSrcB[i];
    }
}

void mat_trans_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst) {
    uint16_t rows = pSrc->numRows;
    uint16_t cols = pSrc->numCols;
    for (uint16_t j = 0; j < cols; j++) {
        for (uint16_t i = 0; i < rows; i++) {
            pDst->pData[j * rows + i] = pSrc->pData[i * cols + j];
        }
    }
}

int mat_cholesky_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst) {
    const int n = (int)pDst->numRows;
    if (pSrc->numRows != pSrc->numCols || pDst->numRows != pDst->numCols || pSrc->numRows != pDst->numRows) {
        return 1;
    }

    // computes lower triangular cholesky
    for (int row = 0; row < n; row++) {
        for (int col = 0; col <= row; col++) {
            float sum = pSrc->pData[row * n + col];
            for (int k = 0; k < col; k++) {
                sum -= pDst->pData[row * n + k] * pDst->pData[col * n + k];
            }

            if (row == col) {
                if (sum <= 0.0F) {
                    return 1; // not positive definite
                }
                pDst->pData[row * n + col] = sqrtf(sum);
            } else {
                pDst->pData[row * n + col] = sum / pDst->pData[col * n + col];
            }
        }

        // Zero the upper triangle for clarity/determinism.
        for (int col = row + 1; col < n; col++) {
            pDst->pData[row * n + col] = 0.0F;
        }
    }

    return 0;
}

void mat_inverse_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst) {
    uint32_t n = pSrc->numRows;
    float *A = pSrc->pData;
    float *inv = pDst->pData;

    uint32_t i, j, k;

    /* Initialize pDst as identity matrix */
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            inv[i*n + j] = (i == j) ? 1.0F : 0.0F;
        }
    }

    /* Copy source matrix into a local working buffer */
    /* NOTE: caller must ensure pDst != pSrc */
    float tmp[n * n];
    for (i = 0; i < n*n; i++) {
        tmp[i] = A[i];
    }

    /* Gauss–Jordan elimination */
    for (i = 0; i < n; i++) {

        float diag = tmp[i*n + i];

        /* Normalize pivot row */
        for (j = 0; j < n; j++) {
            tmp[i*n + j] /= diag;
            inv[i*n + j] /= diag;
        }

        /* Eliminate column i in other rows */
        for (k = 0; k < n; k++) {
            if (k == i) continue;

            float factor = tmp[k*n + i];

            for (j = 0; j < n; j++) {
                tmp[k*n + j] -= factor * tmp[i*n + j];
                inv[k*n + j] -= factor * inv[i*n + j];
            }
        }
    }
}
