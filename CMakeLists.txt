cmake_minimum_required(VERSION 3.22)

# There are two configure areas that are very different:
#   firmware-*  ARM GNU toolchain, add_subdirectory(STM32)
#   host        native gcc/clang, ESKF CTest + protocol check
#
# Firmware unit tests use Ceedling in STM32/tests (`just test-firmware`).


option(FIRM_BUILD_FIRMWARE "Build STM32 firmware (requires the ARM GNU toolchain file)" OFF)

if(FIRM_BUILD_FIRMWARE)
    project(FIRM LANGUAGES C ASM)
    if(NOT CMAKE_SYSTEM_NAME STREQUAL "Generic")
        message(FATAL_ERROR
            "Firmware presets require CMAKE_TOOLCHAIN_FILE=STM32/cmake/gcc-arm-none-eabi.cmake "
            "(or starm-clang). The host preset cannot build the ELF.")
    endif()
    add_subdirectory(STM32)
    return()
endif()

project(FIRM LANGUAGES C)

if(CMAKE_SYSTEM_NAME STREQUAL "Generic")
    message(FATAL_ERROR
        "Host configure loaded an ARM toolchain. Use the `host` preset, or the "
        "`firmware-debug` / `firmware-release` presets for the ELF.")
endif()

include(CTest)
add_subdirectory(processing/firm/eskf_lab/native)

if(BUILD_TESTING)
    add_executable(firm_wire_layout_test tests/protocol/firm_wire_layout_test.c)
    target_include_directories(firm_wire_layout_test PRIVATE
        "${CMAKE_SOURCE_DIR}/STM32/Core/Inc/shared_data"
    )
    add_test(NAME firm_wire_layout_test COMMAND firm_wire_layout_test)
endif()
