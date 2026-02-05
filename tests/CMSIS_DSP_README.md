# CMSIS-DSP Compilation for Tests

To run the tests, you need to pre-compile certain CMSIS-DSP functions. This is automated based on your platform:

## Automatic Compilation

Before running `ceedling test:all`, first run:

**Windows (PowerShell):**
```powershell
.\compile_cmsis_dsp.ps1
```

**Linux/macOS (Bash):**
```bash
./build_cmsis.sh
```

Or use the cross-platform Ruby helper:
```bash
ruby build_helpers/compile_cmsis.rb
```

## What This Does

The script compiles three CMSIS-DSP matrix functions from the STM32 project into object files:
- `arm_mat_trans_f32.c` -> `arm_mat_trans_f32.o`
- `arm_mat_add_f32.c` -> `arm_mat_add_f32.o`
- `arm_mat_scale_f32.c` -> `arm_mat_scale_f32.o`

These object files are then linked during the test build process.

## Adding New Functions

To add more CMSIS-DSP functions:

1. Edit `compile_cmsis_dsp.ps1` (Windows) - add function names to the `$Functions` array
2. Edit `compile_cmsis_dsp.sh` (Unix) - add function names to the `FUNCTIONS` array or pass as arguments
3. Update `build_helpers/compile_cmsis.rb` - add function names to the Ruby functions array

Then update the linker flags in `project.yml` `:flags: :test: :link:` section to include the new `.o` files.

## File Structure

- `compile_cmsis_dsp.ps1` - PowerShell compilation script (Windows)
- `compile_cmsis_dsp.sh` - Bash compilation script (Unix/Linux/macOS)
- `build_helpers/compile_cmsis.rb` - Cross-platform Ruby helper that auto-detects OS
- `build_cmsis.bat` - Windows batch wrapper
- `build_cmsis.sh` - Unix shell wrapper
- `project.yml` - Ceedling configuration (links compiled `.o` files)
