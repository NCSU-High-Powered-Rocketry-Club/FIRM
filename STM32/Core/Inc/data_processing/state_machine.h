#pragma once
#include "eskf_config.h"
#include <string.h>

struct ESKF;

/**
 * @brief Initialise the ESKF flight state to INIT and load the init Q/R diags.
 */
void eskf_state_init(struct ESKF *eskf);

/**
 * @brief Check for INIT → RUNNING transition (time-based).
 *
 * When transitioning, computes fixed IMU biases from accumulated samples.
 */
void eskf_state_update(struct ESKF *eskf);