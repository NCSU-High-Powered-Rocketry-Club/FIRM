# Source and include lists shared by the firmware ELF (STM32/CMakeLists.txt)
# and the host build (root CMakeLists.txt), so both compile the same files.

get_filename_component(FIRM_STM32_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

set(FIRM_ESKF_SOURCES
    "${FIRM_STM32_DIR}/Core/Src/data_processing/matrix_helper.c"
    "${FIRM_STM32_DIR}/Core/Src/data_processing/error_state_kalman_filter.c"
    "${FIRM_STM32_DIR}/Core/Src/data_processing/eskf_functions.c"
    "${FIRM_STM32_DIR}/Core/Src/data_processing/eskf_config.c"
)

# Hardware-independent headers. Host code must not see HAL, FreeRTOS, or the
# SPIUtils/FATFS/W25Q128JV drivers; host tests stub those in STM32/tests/support.
set(FIRM_HOST_INCLUDE_DIRS
    "${FIRM_STM32_DIR}/Core/Inc"
    "${FIRM_STM32_DIR}/Core/Inc/modules"
    "${FIRM_STM32_DIR}/Core/Inc/shared_data"
    "${FIRM_STM32_DIR}/Core/Inc/data_processing"
    "${FIRM_STM32_DIR}/Core/Inc/interfaces"
    "${FIRM_STM32_DIR}/Libraries/BMP581"
    "${FIRM_STM32_DIR}/Libraries/MMC5983MA"
    "${FIRM_STM32_DIR}/Libraries/ICM45686"
    "${FIRM_STM32_DIR}/Libraries/ADXL371"
)
