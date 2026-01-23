# Tests helper

This directory contains helper scripts used to compile a few CMSIS-DSP source files and tables
that are required by the unit tests. Some CMSIS functions are provided as header-only
inline functions or depend on large precomputed tables; to run the tests we compile the
needed implementations into object files and link them with the tests.

Important notes
- You should only run the Ruby helper: `ruby build_helpers/compile_cmsis.rb` from the `tests` directory.
- The Ruby helper is the single source-of-truth for which CMSIS-DSP functions are precompiled.
  Edit `tests/build_helpers/compile_cmsis.rb` to add/remove functions needed by tests.
- The Ruby helper calls the platform-specific script (`compile_cmsis_dsp.sh` on Unix or
  `compile_cmsis_dsp.ps1` on Windows). Do not call those scripts directly; they expect the
  function list to be provided by the Ruby script.

How to use
1. From repository root, run the Ruby helper (it will invoke the appropriate platform script):

```powershell
cd tests
ruby build_helpers/compile_cmsis.rb
```

2. Run the Ceedling tests as usual:

```powershell
ceedling test:all
```

If you add new tests that require additional CMSIS functions or tables, add the corresponding
entries to the `functions` array in `tests/build_helpers/compile_cmsis.rb` and re-run the helper.

Implementation details
- The Ruby helper is responsible for the canonical list of functions to compile. It invokes the
  platform script and passes the function list as arguments.
- The platform scripts compile source files when present and will generate small wrapper C files
  for header-only inline functions when needed.
- Compiled object files are placed under `tests/` in directories matching their CMSIS source tree
  (e.g., `tests/FastMathFunctions/arm_sin_f32.o`).

If you want the scripts to behave differently (e.g., change where object files are written), edit
`tests/build_helpers/compile_cmsis.rb` and the platform scripts accordingly.
