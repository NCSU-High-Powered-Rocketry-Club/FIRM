#!/bin/bash

# Script to pre-compile CMSIS-DSP functions needed for tests
# This script compiles specified CMSIS-DSP source files into object files
# Works on both Unix and Windows
# Usage: ./compile_cmsis_dsp.sh arm_mat_trans_f32 arm_mat_add_f32 arm_mat_scale_f32

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Configuration
OUTPUT_DIR="${SCRIPT_DIR}"
CMSIS_INCLUDE="${SCRIPT_DIR}/../STM32/Drivers/CMSIS-DSP/Include"
CMSIS_ARM_INCLUDE="${SCRIPT_DIR}/../STM32/Drivers/CMSIS/Include"
SOURCE_BASE="${SCRIPT_DIR}/../STM32/Drivers/CMSIS-DSP/Source/MatrixFunctions"

# Parse functions from arguments
FUNCTIONS=("$@")
if [ ${#FUNCTIONS[@]} -eq 0 ]; then
    FUNCTIONS=("arm_mat_trans_f32" "arm_mat_add_f32" "arm_mat_scale_f32")
fi

# Color codes
GREEN='\033[32m'
YELLOW='\033[33m'
RED='\033[31m'
RESET='\033[0m'

echo -e "${YELLOW}Compiling CMSIS-DSP functions...${RESET}"

compiled=0
failed=0

for func in "${FUNCTIONS[@]}"; do
    source_file="${SOURCE_BASE}/${func}.c"
    output_file="${OUTPUT_DIR}/${func}.o"
    
    # Check if source file exists
    if [ ! -f "$source_file" ]; then
        echo -e "${RED}ERROR: Source file not found: $source_file${RESET}"
        ((failed++))
        continue
    fi
    
    # Check if object file is already up-to-date
    if [ -f "$output_file" ] && [ "$source_file" -ot "$output_file" ]; then
        echo -e "${GREEN}[OK]${RESET} ${func}.o (up-to-date)"
        ((compiled++))
        continue
    fi
    
    # Compile the file
    echo "  Compiling ${func}.c..."
    if gcc -I"${CMSIS_INCLUDE}" -I"${CMSIS_ARM_INCLUDE}" -c "$source_file" -o "$output_file" 2>&1; then
        echo -e "${GREEN}[OK]${RESET} ${func}.o"
        ((compiled++))
    else
        echo -e "${RED}[FAIL]${RESET} ${func}.o failed"
        ((failed++))
    fi
done

echo ""
echo -e "Compilation summary: ${GREEN}${compiled} compiled${RESET}, ${RED}${failed} failed${RESET}"

if [ $failed -gt 0 ]; then
    exit 1
fi
exit 0
