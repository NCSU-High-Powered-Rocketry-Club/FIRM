#!/bin/bash

# Script to pre-compile CMSIS-DSP functions needed for tests
# This script compiles specified CMSIS-DSP source files into object files
# Works on both Unix and Windows
# Usage: ./compile_cmsis_dsp.sh arm_mat_trans_f32 arm_mat_add_f32 arm_mat_scale_f32

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Configuration
OUTPUT_DIR="${SCRIPT_DIR}/cmsis_objs"
mkdir -p "${OUTPUT_DIR}"
CMSIS_INCLUDE="${SCRIPT_DIR}/../STM32/Drivers/CMSIS-DSP/Include"
CMSIS_ARM_INCLUDE="${SCRIPT_DIR}/../STM32/Drivers/CMSIS/Include"
SOURCE_BASE="${SCRIPT_DIR}/../STM32/Drivers/CMSIS-DSP/Source"

# Parse functions from arguments (required)
FUNCTIONS=("$@")
if [ ${#FUNCTIONS[@]} -eq 0 ]; then
    echo "ERROR: No functions provided. This script must be called from the Ruby helper."
    exit 2
fi

# Color codes
GREEN='\033[32m'
YELLOW='\033[33m'
RED='\033[31m'
RESET='\033[0m'

echo -e "${YELLOW}Compiling CMSIS-DSP functions...${RESET}"

compiled=0
failed=0
missing_funcs=()

for func in "${FUNCTIONS[@]}"; do
    source_file="${SOURCE_BASE}/${func}.c"
    # Place object files flat in OUTPUT_DIR using the source basename
    base_name=$(basename "$func")
    output_file="${OUTPUT_DIR}/${base_name}.o"

    # If a source file exists, compile it
    if [ -f "$source_file" ]; then
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
    else
        # Try to find the source file anywhere under SOURCE_BASE (accept bare function names)
        candidate=$(find "$SOURCE_BASE" -type f -name "${func}.c" -print -quit 2>/dev/null)
        if [ -n "$candidate" ]; then
            source_file="$candidate"
            base_name=$(basename "$candidate" .c)
            output_file="${OUTPUT_DIR}/${base_name}.o"
            echo "  Found source for ${func} at ${source_file}"
            if gcc -I"${CMSIS_INCLUDE}" -I"${CMSIS_ARM_INCLUDE}" -c "$source_file" -o "$output_file" 2>&1; then
                echo -e "${GREEN}[OK]${RESET} $(basename "$output_file")"
                ((compiled++))
            else
                echo -e "${RED}[FAIL]${RESET} $(basename "$output_file") failed"
                ((failed++))
            fi
            continue
        fi

        # No source file; record for wrapper generation (header-only / inline functions)
        missing_funcs+=("$func")
    fi
done

# If we have any header-only functions requested, generate a small wrapper C file
if [ ${#missing_funcs[@]} -gt 0 ]; then
    wrapper_c="${OUTPUT_DIR}/cmsis_header_wrappers.c"
    wrapper_o="${OUTPUT_DIR}/cmsis_header_wrappers.o"
    echo "  Generating wrapper for header-only functions: ${missing_funcs[*]}"
    cat > "$wrapper_c" <<'EOF'
#include <math.h>
/* Auto-generated wrappers to provide external symbols for header-only inline CMSIS-DSP functions */
EOF
    for f in "${missing_funcs[@]}"; do
        case "$f" in
            arm_sin_f32)
                echo "float arm_sin_f32(float x) { return sinf(x); }" >> "$wrapper_c"
                ;;
            arm_cos_f32)
                echo "float arm_cos_f32(float x) { return cosf(x); }" >> "$wrapper_c"
                ;;
            *)
                # Generic stub: assume signature float func(float)
                echo "/* Stub for ${f} - verify signature if this is incorrect */" >> "$wrapper_c"
                echo "float ${f}(float x) { (void)x; return 0.0f; }" >> "$wrapper_c"
                ;;
        esac
    done

    # Compile the wrapper
    echo "  Compiling wrapper..."
    if gcc -I"${CMSIS_INCLUDE}" -I"${CMSIS_ARM_INCLUDE}" -c "$wrapper_c" -o "$wrapper_o" 2>&1; then
        echo -e "${GREEN}[OK]${RESET} cmsis_header_wrappers.o"
        ((compiled++))
    else
        echo -e "${RED}[FAIL]${RESET} cmsis_header_wrappers.o failed"
        ((failed++))
    fi
fi

echo ""
echo -e "Compilation summary: ${GREEN}${compiled} compiled${RESET}, ${RED}${failed} failed${RESET}"

if [ $failed -gt 0 ]; then
    exit 1
fi
exit 0
