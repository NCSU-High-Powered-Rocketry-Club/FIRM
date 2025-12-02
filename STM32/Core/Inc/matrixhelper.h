#pragma once
#include <arm_math.h>

int symmetric(arm_matrix_instance_f32 *enter_matrix);

void rotvec_to_quat(const float rotvec[3], float quat[4]);