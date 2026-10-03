# FIRM

[![CI](https://github.com/NCSU-High-Powered-Rocketry-Club/FIRM/actions/workflows/ci.yml/badge.svg)](https://github.com/NCSU-High-Powered-Rocketry-Club/FIRM/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.11%2B-blue.svg)](https://www.python.org/)
[![Rust](https://img.shields.io/badge/rust-stable-orange.svg)](https://www.rust-lang.org/)

Filtered Inertial Rotation Module

FIRM is a project by the NCSU High Powered Rocketry Club to develop a flight computer for high powered rockets. The flight computer is designed to provide accurate attitude and position data during flight using an array of sensors and an error-state Kalman filter.

It is also designed to be modular and easily adaptable to different rocket configurations and mission requirements. The project includes a custom PCB that integrates the sensors and microcontroller into a compact, lightweight package suitable for high-speed flight.

This repository holds the STM32 firmware (written in C), a USB client (Rust, with Python and TypeScript bindings) that interfaces with the hardware, and post-flight processing tools for log archives, ESKF replay, and trace analysis.

## Layout

| Path | What it is |
|------|------------|
| `STM32/` | Firmware (CubeMX / VS Code STM32 extension). Keep this name. |
| `client/` | USB client crates and `firm-client` Python bindings |
| `processing/` | Offline tools: `firm-hprc` (`firm.flight_data`, `firm.eskf_lab`) |
| `flight_data/` | Versioned flight-log archive (data only) |
| `tests/` | Pytest and protocol C checks (firmware unit tests live in `STM32/tests/`) |

## Setup

Install [uv](https://docs.astral.sh/uv/), Rust, CMake, Ninja, and [just](https://github.com/casey/just). `just` is a command runner, so every `just <recipe>` below needs it on your `PATH`. Once uv is installed, the easiest way to get it on any OS is:

```bash
uv tool install rust-just
```

Other options are `brew install just` (macOS), `winget install Casey.Just` (Windows), or `cargo install just`. See the [just install docs](https://just.systems/man/en/packages.html) for more. 
Clone the repo and sync Python packages:

```bash
git clone https://github.com/NCSU-High-Powered-Rocketry-Club/FIRM.git
cd FIRM
just sync
```

`just sync` installs `firm-hprc` and `firm-client` into your `.venv`.

## Commands

```bash
just build          # firmware Debug ELF + host ESKF + cargo
just test           # all C tests (CTest) + cargo test + pytest (summary at the end)
just lint           # ruff, rustfmt, clippy, clang-format
just ci             # local firmware/host/rust/python/lint
just --list         # list all just recipes including building/testing/linting individual targets
```

## Processing tools

After running `just sync`:

```bash
uv run firm-log --help
uv run firm-eskf --help
uv run firm-trace -i STM32/trace.bin -o trace.json
uv run firm-reconstruct --help
```

Live USB uses `from firm_client import FIRMClient` (`just sync`), not `from firm import FIRM`.

## Hardware

| Part Number   | Description     | Datasheet                                                                                                   |
|---------------|-----------------|-------------------------------------------------------------------------------------------------------------|
| ICM-45686     | 6 Axis IMU      | [Datasheet](https://www.mouser.com/catalog/specsheets/TDK_DS_000577_ICM_45686.pdf)                          |
| STM32F405RGT6 | Microcontroller | [Datasheet](https://www.st.com/resource/en/datasheet/dm00037051.pdf)                                        |
| MMC5983MA     | Magnetometer    | [Datasheet](https://media.digikey.com/pdf/Data%20Sheets/MEMSIC%20PDFs/MMC5983MA_RevA_4-3-19.pdf)            |
| BMP581        | Pressure Sensor | [Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf) |
