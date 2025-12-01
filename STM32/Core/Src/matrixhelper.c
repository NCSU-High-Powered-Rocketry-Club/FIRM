#include "matrixhelper.h"

int symmetric(arm_matrix_instance_f32 *enter_matrix) {
    uint16_t mat_rows = enter_matrix->numRows;
    uint16_t mat_cols = enter_matrix->numCols;
    float32_t scale_val = 0.5;
    if (mat_rows != mat_cols) {
        return 1;
    }
    
    // Static buffers for temporary matrices (supports up to 21x21 matrices)
    static float32_t temp_transpose[441];
    static float32_t temp_add[441];
    
    

    // Create temporary matrix structures
    arm_matrix_instance_f32 output_matrix;
    output_matrix.numCols = mat_cols;
    output_matrix.numRows = mat_rows;
    output_matrix.pData = temp_transpose;
    
    arm_matrix_instance_f32 output_matrix2;
    output_matrix2.numCols = mat_cols;
    output_matrix2.numRows = mat_rows;
    output_matrix2.pData = temp_add;
    
    // Perform operations: result = 0.5 * (A + A^T)
    arm_mat_trans_f32(enter_matrix, &output_matrix);
    arm_mat_add_f32(enter_matrix, &output_matrix, &output_matrix2);
    arm_mat_scale_f32(&output_matrix2, scale_val, enter_matrix);
}
