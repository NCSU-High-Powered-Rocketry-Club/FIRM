firm_add_c_test(test_wire_layout SOURCES protocol/firm_wire_layout_test.c LABELS protocol)

# Real command dispatch behind a stdin/stdout pipe. Not a CTest test: the pytest
# integration suite drives it (tests/integration/test_get_device_info_pipeline.py).
add_executable(firm_device_info_harness
    integration/stm32_device_info_harness.c
    "${FIRM_STM32_DIR}/Core/Src/modules/commands.c"
)
target_link_libraries(firm_device_info_harness PRIVATE firm_test_common)
set_target_properties(firm_device_info_harness PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
