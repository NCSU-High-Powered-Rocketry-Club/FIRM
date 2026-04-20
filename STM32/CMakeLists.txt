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

set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/Src/firm_tasks.c)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

target_compile_options(${PROJECT_NAME} PRIVATE
    -Wa,-adhlns=${PROJECT_NAME}.lst
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -Ofast
)

target_link_options(${PROJECT_NAME} PRIVATE
    -u _printf_float
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -Ofast
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
    Libraries/ADXL371/adxl371.c
    Libraries/SPIUtils/spi_utils.c
    Libraries/FATFS_SD/fatfs_sd.c

    Core/Src/tasks/sensor_task.c
    Core/Src/tasks/usb_read_data_task.c
    
    Core/Src/data_processing/matrix_helper.c
    Core/Src/data_processing/error_state_kalman_filter.c
    Core/Src/data_processing/eskf_functions.c
    Core/Src/data_processing/eskf_config.c
    Core/Src/data_processing/clock_cycle_count.c
    Core/Src/data_processing/mocking_ring_buffer.c

    Core/Src/interfaces/settings_storage.c
    Core/Src/interfaces/logger_storage.c

    Core/Src/modules/settings_manager.c
    Core/Src/modules/sensor_manager.c
    Core/Src/modules/logger.c
    Core/Src/modules/commands.c
    Core/Src/modules/messages.c
    Core/Src/modules/mocking_handler.c

    Core/Src/target/firm_v1_0.c

    Core/Src/freertos_trace.c
    Core/Src/firm_tasks.c
    Core/Src/led.c
    Core/Src/firm_fsm.c
    
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    Core/Inc/tasks
    Core/Inc/data_processing
    Core/Inc/target
    Core/Inc/interfaces
    Core/Inc/modules
    Core/Inc/shared_data
    Libraries/ICM45686
    Libraries/BMP581
    Libraries/MMC5983MA
    Libraries/W25Q128JV
    Libraries/SPIUtils
    Libraries/ADXL371
    Libraries/FATFS_SD
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    ARM_MATH_LOOPUNROLL
    STM32F405xx
    ARM_MATH_CM4
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m

    # Add user defined libraries
)
