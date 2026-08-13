cmake_minimum_required(VERSION 3.22)
project(firm_eskf_host C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

get_filename_component(FIRM_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(STM32_ROOT "${FIRM_ROOT}/STM32")

add_library(firm_eskf_core STATIC
    "${STM32_ROOT}/Core/Src/data_processing/error_state_kalman_filter.c"
    "${STM32_ROOT}/Core/Src/data_processing/eskf_functions.c"
    "${STM32_ROOT}/Core/Src/data_processing/eskf_config.c"
    "${STM32_ROOT}/Core/Src/data_processing/matrix_helper.c"
    src/settings_manager_host.c
)

target_include_directories(firm_eskf_core PUBLIC
    include
    "${STM32_ROOT}/Core/Inc"
    "${STM32_ROOT}/Core/Inc/data_processing"
    "${STM32_ROOT}/Core/Inc/interfaces"
    "${STM32_ROOT}/Core/Inc/modules"
    "${STM32_ROOT}/Core/Inc/shared_data"
)

if(MSVC)
    target_compile_options(firm_eskf_core PRIVATE /O2 /fp:fast /W4)
else()
    target_compile_options(firm_eskf_core PRIVATE -O3 -ffast-math -march=native -Wall -Wextra)
    target_link_libraries(firm_eskf_core PUBLIC m)
endif()

add_executable(firm_eskf_replay src/eskf_replay.c)
target_link_libraries(firm_eskf_replay PRIVATE firm_eskf_core)

include(CTest)
if(BUILD_TESTING)
    add_executable(firm_eskf_smoke_test tests/eskf_smoke_test.c)
    target_link_libraries(firm_eskf_smoke_test PRIVATE firm_eskf_core)
    add_test(NAME firm_eskf_smoke_test COMMAND firm_eskf_smoke_test)

    add_executable(firm_eskf_math_test tests/eskf_math_test.c)
    target_link_libraries(firm_eskf_math_test PRIVATE firm_eskf_core)
    add_test(NAME firm_eskf_math_test COMMAND firm_eskf_math_test)
endif()
