# Host Filter Testing Foundation

This folder contains a desktop harness for the ESKF in STM32 data_processing, driven by Python.

## Purpose

- Compile the portable filter C code as a shared library on desktop.
- Load the shared library from Python using ctypes.
- Run a smoke path: initialize, step, read state, reset, reinitialize.

## Included C Sources

The desktop build includes only:

- STM32/Core/Src/data_processing/matrix_helper.c
- STM32/Core/Src/data_processing/eskf_functions.c
- STM32/Core/Src/data_processing/eskf_config.c
- STM32/Core/Src/data_processing/error_state_kalman_filter.c
- python/filter_testing/setup.c

Excluded intentionally:

- STM32/Core/Src/data_processing/data_preprocess.c

## Prerequisites

- Windows with gcc in PATH (MinGW/MSYS2).
- Python environment active (repo .venv is fine).

## Quick Start

From repo root:

```powershell
c:/Users/Wlsan/git/FIRM/.venv/Scripts/python.exe python/filter_testing/run_smoke.py
```

This command will:

1. Build or reuse the shared library.
2. Run a stationary-data smoke test.
3. Print state outputs at selected steps.
4. Verify reset and reinitialize in the same process.

## Build Only

```powershell
c:/Users/Wlsan/git/FIRM/.venv/Scripts/python.exe python/filter_testing/build_filter.py --verbose
```

Use `--force` to rebuild even when up to date.

## Python API Files

- build_filter.py: compile/link orchestration for shared library.
- filter_api.py: ctypes bindings and class wrapper for setup.c API.
- run_smoke.py: minimal executable smoke flow.

## Deferred Scope

Not included yet:

- Dataset reading/parsing.
- Routing full output traces back to Python plotting.
- Accuracy or regression metric validation.

Those can be layered on top of this API without changing the build foundation.
