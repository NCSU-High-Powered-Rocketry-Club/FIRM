# FIRM

Filtered Inertial Rotation Module — a flight computer for the NCSU High Powered Rocketry Club.

This repository holds STM32 firmware, a USB client (Rust / Python / TypeScript), and post-flight processing tools.

## Layout

| Path | What it is |
|------|------------|
| `STM32/` | Firmware (CubeMX / VS Code STM32 extension). Keep this name. |
| `client/` | USB client crates and `firm-client` Python bindings |
| `processing/` | Offline tools: `firm-hprc` (`firm` + `eskf_lab`) |
| `flight_data/` | Versioned flight-log archive (data only) |
| `tests/` | Pytest, protocol C checks. Firmware Unity tests stay in `STM32/tests/` |

## Setup

Install [uv](https://docs.astral.sh/uv/), Rust, CMake, Ninja, and [just](https://github.com/casey/just). Clone the repo and sync Python packages:

```bash
git clone https://github.com/NCSU-High-Powered-Rocketry-Club/FIRM.git
cd FIRM
just sync
```

`just sync` installs `firm-hprc` and `firm-client` (USB extra) into `.venv`.

## Commands

```bash
just build          # firmware Debug ELF + host ESKF + cargo
just test           # Ceedling + host CTest + cargo test + pytest (summary at the end)
just lint           # ruff, rustfmt, clippy, clang-format
just ci             # sequential local coverage matching GitHub Actions
```

Useful splits:

```bash
just build-firmware     # cmake --preset firmware-debug
just test-host          # ESKF CTest + C header layout (not Ceedling)
just test-firmware      # ceedling test:all in STM32/tests (needs Ruby + Ceedling)
just test-python        # pytest, excluding @pytest.mark.integration
just test-integration   # Node + WASM pipeline tests
```

CI runs the same recipes as parallel jobs. Use `just --list` for the full set.

## CMake presets

Configure from the **repository root**:

- `firmware-debug` / `firmware-release` — ARM GNU, output in `build/firmware-*`
- `host` — native ESKF replay and tests, output in `build/host`

CLion: one CMake profile per preset. Do not point CMake at `STM32/` unless you are using the ST VS Code / CubeMX standalone project (`STM32/CMakePresets.json` is kept for that).

## Processing tools

After `just sync`:

```bash
uv run firm-log --help
uv run firm-eskf --help
uv run firm-trace -i STM32/trace.bin -o trace.json
uv run firm-reconstruct --help
```

Live USB uses `from firm_client import FIRMClient` (`uv sync --extra usb`), not `from firm import FIRM`.

## Hardware

| Part Number    | Description       | Datasheet |
|----------------|-------------------|-----------|
| ICM-45686      | 6 Axis IMU        | [Datasheet](https://www.mouser.com/catalog/specsheets/TDK_DS_000577_ICM_45686.pdf) |
| STM32F405RGT6  | Microcontroller   | [Datasheet](https://www.st.com/resource/en/datasheet/dm00037051.pdf) |
| MMC5983MA      | Magnetometer      | [Datasheet](https://media.digikey.com/pdf/Data%20Sheets/MEMSIC%20PDFs/MMC5983MA_RevA_4-3-19.pdf) |
| BMP581         | Pressure Sensor   | [Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf) |
