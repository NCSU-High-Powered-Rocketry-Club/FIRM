#!/bin/bash
# Build helper script for Unix - compiles CMSIS-DSP functions before tests
ruby "$(dirname "$0")/build_helpers/compile_cmsis.rb"
exit $?
