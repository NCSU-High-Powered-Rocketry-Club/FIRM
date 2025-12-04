#include "matrixhelper.h"

int symmetric(arm_matrix_instance_f64 *enter_matrix) {
    uint16_t mat_rows = enter_matrix->numRows;
    uint16_t mat_cols = enter_matrix->numCols;
    double scale_val = 0.5;
    if (mat_rows != mat_cols) {
        return 1;
    }
    
    for (int row = 0; row < mat_rows; row++) {
        for (int col = row + 1; col < mat_cols; col++) {
            double right_val = enter_matrix->pData[row * mat_cols + col];
            double left_val = enter_matrix->pData[col * mat_cols + row];
            double avg = 0.5 * (right_val + left_val);
            enter_matrix->pData[row * mat_cols + col] = avg;
            enter_matrix->pData[col * mat_cols + row] = avg;
        }
    }
    
    return 0;
}

void rotvec_to_quat(const double rotvec[3], double quat[4]) {
    double theta;
    theta = sqrt(rotvec[0] * rotvec[0] + rotvec[1] * rotvec[1] + rotvec[2] * rotvec[2]);
    theta *= 0.5;
    if (theta == 0.0) {
        quat[0] = 1.0;
        quat[1] = 0.0;
        quat[2] = 0.0;
        quat[3] = 0.0;
        return;
    }

    double s = sin(theta) / theta;
    quat[0] = cos(theta);
    quat[1] = (rotvec[0] * 0.5) * s;
    quat[2] = (rotvec[1] * 0.5) * s;
    quat[3] = (rotvec[2] * 0.5) * s;
}

void quat_to_rotvec(const double quat[4], double rotvec[3]) {
    // Normalize quaternion manually (no arm_quaternion_norm_f64 available)
    double quat_norm = sqrt(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);

    double w = quat[0] / quat_norm;
    double x = quat[1] / quat_norm;
    double y = quat[2] / quat_norm;
    double z = quat[3] / quat_norm;

    double half_angle = acos(w);
    double theta = 2.0 * half_angle;
    double sin_half = sin(half_angle);
    if (fabs(sin_half) < 1e-6) {
        rotvec[0] = 0.0;
        rotvec[1] = 0.0;
        rotvec[2] = 0.0;
        return;
    }

    double scale = theta / sin_half;
    rotvec[0] = x * scale;
    rotvec[1] = y * scale;
    rotvec[2] = z * scale;
}

// Wrapper implementations for CMSIS-DSP functions that don't have f64 versions
// These implement double-precision versions using manual algorithms or wrapping f32 functions

// Scale matrix: pDst = scale * pSrc
void arm_mat_scale_f64_wrapper(const arm_matrix_instance_f64 *pSrc, double scale, arm_matrix_instance_f64 *pDst) {
    uint32_t numElements = pSrc->numRows * pSrc->numCols;
    for (uint32_t i = 0; i < numElements; i++) {
        pDst->pData[i] = pSrc->pData[i] * scale;
    }
}

// Add matrices: pDst = pSrcA + pSrcB
void arm_mat_add_f64_wrapper(const arm_matrix_instance_f64 *pSrcA, const arm_matrix_instance_f64 *pSrcB, arm_matrix_instance_f64 *pDst) {
    uint32_t numElements = pSrcA->numRows * pSrcA->numCols;
    for (uint32_t i = 0; i < numElements; i++) {
        pDst->pData[i] = pSrcA->pData[i] + pSrcB->pData[i];
    }
}


// Subtract vectors: pDst[i] = pSrcA[i] - pSrcB[i]
void arm_sub_f64_wrapper(const double *pSrcA, const double *pSrcB, double *pDst, uint32_t blockSize) {
    for (uint32_t i = 0; i < blockSize; i++) {
        pDst[i] = pSrcA[i] - pSrcB[i];
    }
}

// Multiply vectors element-wise: pDst[i] = pSrcA[i] * pSrcB[i]
void arm_mult_f64_wrapper(const double *pSrcA, const double *pSrcB, double *pDst, uint32_t blockSize) {
    for (uint32_t i = 0; i < blockSize; i++) {
        pDst[i] = pSrcA[i] * pSrcB[i];
    }
}

// Quaternion norm
void arm_quaternion_norm_f64_wrapper(const double *pInputQ, double *pNorm, uint32_t numQ) {
    (void)numQ;  // Unused - we only handle single quaternion
    *pNorm = sqrt(pInputQ[0] * pInputQ[0] + pInputQ[1] * pInputQ[1] + 
                  pInputQ[2] * pInputQ[2] + pInputQ[3] * pInputQ[3]);
}

// Quaternion product (single precision version)
// out = q1 * q2 (Hamilton product)
void arm_quaternion_product_single_f64_wrapper(const double *q1, const double *q2, double *out) {
    double w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
    double w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];
    
    out[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    out[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    out[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    out[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2;
}