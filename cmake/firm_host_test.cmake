# Every host C test links firm_test_common: utest.h, the shared STM32 include
# list, and one set of warnings. utest.h registers tests itself, so a test is
# just an executable that CTest runs.

add_library(firm_test_common INTERFACE)
target_include_directories(firm_test_common INTERFACE
    "${CMAKE_SOURCE_DIR}/third_party/utest"
    ${FIRM_HOST_INCLUDE_DIRS}
)
target_compile_definitions(firm_test_common INTERFACE TEST)
if(NOT MSVC)
    # utest's setup/teardown hooks pass utest_result/utest_fixture/utest_index,
    # which most tests ignore.
    target_compile_options(firm_test_common INTERFACE -Wall -Wextra -Wno-unused-parameter)
    target_link_libraries(firm_test_common INTERFACE m)
endif()

# firm_add_c_test(<name> SOURCES <files...> [LIBS <targets...>] [LABELS <labels...>])
function(firm_add_c_test name)
    cmake_parse_arguments(PARSE_ARGV 1 ARG "" "" "SOURCES;LIBS;LABELS")
    add_executable(${name} ${ARG_SOURCES})
    target_link_libraries(${name} PRIVATE firm_test_common ${ARG_LIBS})
    add_test(NAME ${name} COMMAND ${name})
    if(ARG_LABELS)
        set_tests_properties(${name} PROPERTIES LABELS "${ARG_LABELS}")
    endif()
endfunction()
