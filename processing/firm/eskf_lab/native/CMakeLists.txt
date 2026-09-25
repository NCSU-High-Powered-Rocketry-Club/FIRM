# Host ESKF library and replay tool. Configured from the repo root (`host` preset).

add_library(firm_eskf_core STATIC
    ${FIRM_ESKF_SOURCES}
    src/settings_manager_host.c
)

target_include_directories(firm_eskf_core PUBLIC
    include
    ${FIRM_HOST_INCLUDE_DIRS}
)

if(MSVC)
    target_compile_options(firm_eskf_core PRIVATE /O2 /fp:fast /W4)
else()
    target_compile_options(firm_eskf_core PRIVATE -O3 -ffast-math -march=native -Wall -Wextra)
    target_link_libraries(firm_eskf_core PUBLIC m)
endif()

add_executable(firm_eskf_replay src/eskf_replay.c)
target_link_libraries(firm_eskf_replay PRIVATE firm_eskf_core)
# Fixed location so `firm-eskf` finds the replay in <build dir>/bin.
set_target_properties(firm_eskf_replay PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

if(BUILD_TESTING)
    firm_add_c_test(firm_eskf_smoke_test SOURCES tests/eskf_smoke_test.c LIBS firm_eskf_core LABELS eskf)
    firm_add_c_test(firm_eskf_math_test SOURCES tests/eskf_math_test.c LIBS firm_eskf_core LABELS eskf)
endif()
