#!/usr/bin/env ruby

# Cross-platform helper to compile CMSIS-DSP functions
# Automatically detects Windows vs Unix and runs appropriate script

require 'rbconfig'

# Get the parent directory (tests root)
script_dir = File.expand_path(File.dirname(__FILE__))
tests_root = File.dirname(script_dir)
# Canonical list of CMSIS-DSP functions / files to compile for tests.
# Edit this list when tests require additional CMSIS functions or tables.
functions = [
  'CommonTables/arm_common_tables',
  'FastMathFunctions/arm_sin_f32',
  'FastMathFunctions/arm_cos_f32',
  'MatrixFunctions/arm_mat_trans_f32',
  'MatrixFunctions/arm_mat_add_f32',
  'MatrixFunctions/arm_mat_scale_f32',
  'MatrixFunctions/arm_mat_cholesky_f32',
  'QuaternionMathFunctions/arm_quaternion_norm_f32',
  'QuaternionMathFunctions/arm_quaternion_product_single_f32',
  'QuaternionMathFunctions/arm_quaternion_conjugate_f32'
]

# Note: This script intentionally ignores command-line args. The test helper
# is the single source-of-truth for which CMSIS objects are required.

if RbConfig::CONFIG['host_os'] =~ /mswin|mingw/
  # Windows - use PowerShell
  ps_script = File.join(tests_root, 'compile_cmsis_dsp.ps1')
  if File.exist?(ps_script)
    # Change to tests directory first
    Dir.chdir(tests_root)
    # Call PowerShell script with functions as positional arguments
    cmd = ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", ps_script] + functions
    system(*cmd)
    exit($?.exitstatus || 0)
  end
else
  # Unix/Linux/macOS - use bash
  bash_script = File.join(tests_root, 'compile_cmsis_dsp.sh')
  if File.exist?(bash_script)
    # Change to tests directory first
    Dir.chdir(tests_root)
    # Call bash script directly with array of arguments
    system("bash", bash_script, *functions)
    exit($?.exitstatus || 0)
  end
end

puts "Error: No compilation script found"
exit 1
