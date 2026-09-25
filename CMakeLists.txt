cmake_minimum_required(VERSION 3.22)

# There are two configure areas that are very different:
#   firmware-*  ARM GNU toolchain, add_subdirectory(STM32)
#   host        native gcc/clang, every C test (CTest + utest.h)


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

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

include(CTest)
include(STM32/cmake/firm_sources.cmake)
include(cmake/firm_host_test.cmake)

add_subdirectory(processing/firm/eskf_lab/native)

if(BUILD_TESTING)
    add_subdirectory(STM32/tests)
    add_subdirectory(tests)
endif()
