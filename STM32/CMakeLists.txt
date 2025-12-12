cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME FIRM)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

target_link_options(${PROJECT_NAME} PRIVATE
    -u _printf_float
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Libraries/BMP581/bmp581.c
    Libraries/W25Q128JV/w25q128jv.c
    Libraries/MMC5983MA/mmc5983ma.c
    Libraries/ICM45686/icm45686.c
    Libraries/SPIUtils/spi_utils.c
    Core/Src/logger.c
    Core/Src/usb_print_debug.c
    Core/Src/data_processing/data_preprocess.c
    Core/Src/settings.c
    Core/Src/usb_serializer.c
    Core/Src/firm.c
    Core/Src/led.c
    Core/Src/matrixhelper.c
    Core/Src/data_processing/unscented_kalman_filter.c
    Core/Src/data_processing/ukf_functions.c
    Core/Src/data_processing/state_machine.c
    Core/Src/kalman_filter_config.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
   Core/Inc
   Drivers/CMSIS-DSP/Include
   Core/Inc/data_processing
   Libraries/ICM45686
   Libraries/BMP581
   Libraries/MMC5983MA
   Libraries/W25Q128JV
   Libraries/SPIUtils
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

set(CMSISDSP Drivers/CMSIS-DSP)
set(CMSISCORE "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS")
add_subdirectory(${CMSISDSP}/Source bin_dsp)
target_link_libraries(${CMAKE_PROJECT_NAME} CMSISDSP)