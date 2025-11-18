#include <arm_math.h>

void symmetric(arm_matrix_instance_f32 *enter_matrix){
    uint16_t mat_rows = enter_matrix->numRows;
    uint16_t mat_cols = enter_matrix->numCols;
    uint16_t indicator = 1;
    float32_t scale_val = .5;
    arm_matrix_instance_f32 output_matrix, *ptr_output_matrix;
    ptr_output_matrix = &output_matrix;

    if (mat_rows != mat_cols){
        indicator = 0;
    }

    if (indicator){
        arm_mat_trans_f32(enter_matrix, ptr_output_matrix);
        arm_mat_add_f32(enter_matrix, ptr_output_matrix, ptr_output_matrix);
        arm_mat_scale_f32 (ptr_output_matrix, scale_val, ptr_output_matrix);
    }
    
}
