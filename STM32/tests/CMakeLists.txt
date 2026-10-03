# Firmware unit tests, built for the host. Each test links only the production
# files it exercises; everything else is faked in the test file or in support/.
# support/ provides host stand-ins for the HAL and SPIUtils headers.

set(SRC "${FIRM_STM32_DIR}/Core/Src")
set(LIB "${FIRM_STM32_DIR}/Libraries")
set(HAL_STUBS support/hal_stubs.c support/spi_utils.c)

include_directories(support)

function(firm_add_firmware_test name)
    firm_add_c_test(${name} ${ARGN} LABELS firmware)
endfunction()

# Tests backed by support/fake_logger.c write into their own scratch directory.
function(firm_add_logger_test name)
    firm_add_firmware_test(${name} ${ARGN})
    target_sources(${name} PRIVATE support/fake_logger.c)
    set(log_dir "${CMAKE_CURRENT_BINARY_DIR}/logs/${name}")
    file(MAKE_DIRECTORY "${log_dir}")
    target_compile_definitions(${name} PRIVATE FAKE_LOG_DIR="${log_dir}")
endfunction()

firm_add_firmware_test(test_clock_cycle_count SOURCES
    data_processing/test_clock_cycle_count.c
    ${SRC}/data_processing/clock_cycle_count.c
)
firm_add_firmware_test(test_mocking_ring_buffer SOURCES
    data_processing/test_mocking_ring_buffer.c
    ${SRC}/data_processing/mocking_ring_buffer.c
)

firm_add_firmware_test(test_adxl371 SOURCES drivers/test_adxl371.c ${LIB}/ADXL371/adxl371.c ${HAL_STUBS})
firm_add_firmware_test(test_bmp581 SOURCES drivers/test_bmp581.c ${LIB}/BMP581/bmp581.c ${HAL_STUBS})
firm_add_firmware_test(test_icm45686 SOURCES drivers/test_icm45686.c ${LIB}/ICM45686/icm45686.c ${HAL_STUBS})
firm_add_firmware_test(test_mmc5983ma SOURCES drivers/test_mmc5983ma.c ${LIB}/MMC5983MA/mmc5983ma.c ${HAL_STUBS})

firm_add_logger_test(test_logger_storage SOURCES
    interfaces/test_logger_storage.c
    ${SRC}/interfaces/logger_storage.c
)
firm_add_logger_test(test_logger SOURCES
    modules/test_logger.c
    ${SRC}/modules/logger.c
    ${SRC}/interfaces/logger_storage.c
)

# Settings and mocking-handler dependencies are stubbed in the test file.
firm_add_firmware_test(test_commands SOURCES
    modules/test_commands.c
    ${SRC}/modules/commands.c
)
# dispatch_command/dispatch_mock_msg are faked in the test file.
firm_add_firmware_test(test_messages SOURCES
    modules/test_messages.c
    ${SRC}/modules/messages.c
)
# The mock ring and clock cycle counter are faked in the test file.
firm_add_firmware_test(test_mocking_handler SOURCES
    modules/test_mocking_handler.c
    ${SRC}/modules/mocking_handler.c
    ${SRC}/modules/system_state.c
)
