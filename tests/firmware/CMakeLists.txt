include(FetchContent)

FetchContent_Declare(
    unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG v2.6.1
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(unity)

if(NOT TARGET unity)
    add_library(unity STATIC "${unity_SOURCE_DIR}/src/unity.c")
    target_include_directories(unity PUBLIC "${unity_SOURCE_DIR}/src")
endif()
target_compile_definitions(unity PUBLIC UNITY_INCLUDE_DOUBLE)

set(STM32_ROOT "${CMAKE_SOURCE_DIR}/STM32")
set(FIRM_UNITY_RUNNER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/generate_unity_runner.cmake")

add_library(firm_unity_test_config INTERFACE)
target_include_directories(firm_unity_test_config INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/support"
    "${STM32_ROOT}/Core/Inc"
    "${STM32_ROOT}/Core/Inc/modules"
    "${STM32_ROOT}/Core/Inc/shared_data"
    "${STM32_ROOT}/Core/Inc/data_processing"
    "${STM32_ROOT}/Core/Inc/interfaces"
    "${STM32_ROOT}/Libraries/BMP581"
    "${STM32_ROOT}/Libraries/MMC5983MA"
    "${STM32_ROOT}/Libraries/ICM45686"
    "${STM32_ROOT}/Libraries/ADXL371"
)
target_compile_definitions(firm_unity_test_config INTERFACE TEST)
target_link_libraries(firm_unity_test_config INTERFACE unity)
if(NOT MSVC)
    target_link_libraries(firm_unity_test_config INTERFACE m)
endif()

add_library(firm_hal_stubs STATIC
    support/hal_stubs.c
    support/spi_utils.c
)
target_link_libraries(firm_hal_stubs PUBLIC firm_unity_test_config)

add_library(firm_fake_logger STATIC support/fake_logger.c)
target_link_libraries(firm_fake_logger PUBLIC firm_unity_test_config)

function(add_firm_unity_test test_name)
    cmake_parse_arguments(ARG "" "TEST_FILE" "SOURCES;LIBS" ${ARGN})
    if(NOT ARG_TEST_FILE)
        message(FATAL_ERROR "add_firm_unity_test(${test_name}) requires TEST_FILE")
    endif()

    set(runner "${CMAKE_CURRENT_BINARY_DIR}/${test_name}_runner.c")
    add_custom_command(
        OUTPUT "${runner}"
        COMMAND "${CMAKE_COMMAND}"
            "-DTEST_FILE=${ARG_TEST_FILE}"
            "-DRUNNER_FILE=${runner}"
            -P "${FIRM_UNITY_RUNNER_SCRIPT}"
        DEPENDS "${ARG_TEST_FILE}" "${FIRM_UNITY_RUNNER_SCRIPT}"
        VERBATIM
    )

    add_executable(${test_name}
        "${ARG_TEST_FILE}"
        "${runner}"
        ${ARG_SOURCES}
    )
    target_link_libraries(${test_name} PRIVATE firm_unity_test_config ${ARG_LIBS})

    set(run_dir "${CMAKE_CURRENT_BINARY_DIR}/run/${test_name}")
    file(MAKE_DIRECTORY "${run_dir}")
    add_test(NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY "${run_dir}")
    set_tests_properties(${test_name} PROPERTIES LABELS firmware)
endfunction()

add_firm_unity_test(firmware_test_messages
    TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/modules/test_messages.c"
    SOURCES "${STM32_ROOT}/Core/Src/modules/messages.c"
)

add_firm_unity_test(firmware_test_commands
    TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/modules/test_commands.c"
    SOURCES "${STM32_ROOT}/Core/Src/modules/commands.c"
)

add_firm_unity_test(firmware_test_clock_cycle_count
    TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/data_processing/test_clock_cycle_count.c"
    SOURCES "${STM32_ROOT}/Core/Src/data_processing/clock_cycle_count.c"
)

add_firm_unity_test(firmware_test_mocking_handler
    TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/modules/test_mocking_handler.c"
    SOURCES
        "${STM32_ROOT}/Core/Src/modules/mocking_handler.c"
        "${STM32_ROOT}/Core/Src/modules/system_state.c"
)

add_firm_unity_test(firmware_test_logger
    TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/modules/test_logger.c"
    SOURCES
        "${STM32_ROOT}/Core/Src/modules/logger.c"
        "${STM32_ROOT}/Core/Src/interfaces/logger_storage.c"
    LIBS firm_fake_logger
)

add_firm_unity_test(firmware_test_logger_storage
    TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/interfaces/test_logger_storage.c"
    SOURCES "${STM32_ROOT}/Core/Src/interfaces/logger_storage.c"
    LIBS firm_fake_logger
)

add_firm_unity_test(firmware_test_mocking_ring_buffer
    TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/data_processing/test_mocking_ring_buffer.c"
    SOURCES "${STM32_ROOT}/Core/Src/data_processing/mocking_ring_buffer.c"
)

add_firm_unity_test(firmware_test_bmp581
    TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/drivers/test_bmp581.c"
    SOURCES "${STM32_ROOT}/Libraries/BMP581/bmp581.c"
    LIBS firm_hal_stubs
)

add_firm_unity_test(firmware_test_icm45686
    TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/drivers/test_icm45686.c"
    SOURCES "${STM32_ROOT}/Libraries/ICM45686/icm45686.c"
    LIBS firm_hal_stubs
)

add_firm_unity_test(firmware_test_mmc5983ma
    TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/drivers/test_mmc5983ma.c"
    SOURCES "${STM32_ROOT}/Libraries/MMC5983MA/mmc5983ma.c"
    LIBS firm_hal_stubs
)

add_firm_unity_test(firmware_test_adxl371
    TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/drivers/test_adxl371.c"
    SOURCES "${STM32_ROOT}/Libraries/ADXL371/adxl371.c"
    LIBS firm_hal_stubs
)
