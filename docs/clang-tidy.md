# clang-tidy Integration

This document describes the clang-tidy integration for the FIRM project.

## Overview

clang-tidy is a static analysis tool that provides linting and checks for C/C++ code. It helps catch bugs, improve code quality, and enforce coding standards.

## Configuration

The clang-tidy configuration is stored in `.clang-tidy` at the root of the repository. The configuration includes:

- **Bug-prone checks**: Detects common programming errors
- **Performance checks**: Identifies performance issues
- **Readability checks**: Improves code readability
- **Portability checks**: Ensures code portability
- **Clang analyzer**: Advanced static analysis

The configuration is tuned for embedded STM32 development and excludes checks that are too strict for HAL drivers.

## Pre-commit Hook

clang-tidy is integrated with pre-commit and will run automatically on user-written files when you commit changes.

To set up pre-commit:

```bash
uv sync
pre-commit install
```

## Manual Usage

To run clang-tidy manually on all user files:

```bash
uv run pre-commit run clang-tidy --all-files
```

To run on specific files:

```bash
uv run pre-commit run clang-tidy --files STM32/Core/Src/bmp581.c
```

Or directly using the wrapper script:

```bash
.githooks/run-clang-tidy.sh STM32/Core/Src/bmp581.c
```

## Files Checked

The pre-commit hook is configured to only check user-written files in `STM32/Core/Src/` and `STM32/Core/Inc/`, excluding auto-generated files from STM32CubeIDE.

Currently checked files:
- bmp581.c/h
- firm_utils.c/h
- icm45686.c/h
- logger.c/h
- mmc5983ma.c/h
- spi_utils.c/h
- usb_print_debug.c/h
- packets.h

## Troubleshooting

If clang-tidy reports errors that are not relevant:

1. Check if the error is in auto-generated code (these should be excluded)
2. Verify the `.clang-tidy` configuration is appropriate
3. Consider disabling specific checks if they are too strict

## System Requirements

- clang-tidy must be installed on your system
- On Ubuntu/Debian: `apt install clang-tidy`
- On macOS: `brew install llvm` (includes clang-tidy)
- On Windows: Install LLVM from the official website
