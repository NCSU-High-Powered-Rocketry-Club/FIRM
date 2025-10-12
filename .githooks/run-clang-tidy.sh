#!/usr/bin/env bash
# Wrapper script for running clang-tidy on STM32 project files
# This script provides the necessary compiler flags for clang-tidy to work

set -e

# Base directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR/.."
STM32_DIR="$PROJECT_DIR/STM32"

# Common compiler flags for STM32F405
COMMON_FLAGS=(
    "-DDEBUG"
    "-DSTM32F405xx"
    "-DUSE_HAL_DRIVER"
    "-I$STM32_DIR/Core/Inc"
    "-I$STM32_DIR/Drivers/STM32F4xx_HAL_Driver/Inc"
    "-I$STM32_DIR/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"
    "-I$STM32_DIR/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
    "-I$STM32_DIR/Drivers/CMSIS/Include"
    "-I$STM32_DIR/FATFS/Target"
    "-I$STM32_DIR/FATFS/App"
    "-I$STM32_DIR/USB_DEVICE/App"
    "-I$STM32_DIR/USB_DEVICE/Target"
    "-I$STM32_DIR/Middlewares/Third_Party/FatFs/src"
    "-I$STM32_DIR/Middlewares/ST/STM32_USB_Device_Library/Core/Inc"
    "-I$STM32_DIR/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"
    "-std=gnu11"
)

# Run clang-tidy with the provided arguments and common flags
clang-tidy --config-file="$PROJECT_DIR/.clang-tidy" "$@" -- "${COMMON_FLAGS[@]}"

