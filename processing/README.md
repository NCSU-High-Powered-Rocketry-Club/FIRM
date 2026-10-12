# Processing (`firm-hprc`)

Offline FIRM tools:

- log archive management
- ESKF replay
- FreeRTOS trace conversion
- IMU fixture calibration (needs a connected FIRM)

Install from this repository with `just sync` (or `uv sync --all-packages --extra usb --group dev` at the repo root). 

## Commands

| Command            | Purpose                                                          |
|--------------------|------------------------------------------------------------------|
| `firm-log`         | Ingest, trim, calibrate, build, and export versioned flight logs |
| `firm-eskf`        | Replay production ESKF C sources against built recordings        |
| `firm-imu-cal`     | Calibrate a connected FIRM's IMU on the 26-sided fixture         |
| `firm-trace`       | Convert a GDB `trace.bin` dump to JSON for Spall or Perfetto     |
| `firm-reconstruct` | Recover a v1.4 `.frm` from legacy decoder CSVs                   |

```bash
uv run firm-log ingest --launch my-launch --recording primary --flight LOG1.FRM --hardware new
uv run firm-log build my-launch/primary
uv run firm-log trim preview my-launch/primary --phases
uv run firm-log calibration show my-launch/primary
uv run firm-eskf run my-launch/primary
uv run firm-trace -i STM32/trace.bin -o trace.json
```

### IMU fixture calibration

`firm-imu-cal` fits accelerometer and gyroscope-bias calibration over USB, so it needs the `usb` extra (included in `just sync`):

```bash
uv run firm-imu-cal COM12
```

Replace `COM12` with the device's serial port. The utility checks for identity IMU calibration, guides the operator through all 26 numbered fixture sides, prints the fitted values, and asks before writing them to the device. It does not save captured samples or calibration results to disk.

See [firm/eskf_lab/README.md](firm/eskf_lab/README.md) for information on how to use the filter lab workflow.

## Publishing

Bump `version` in `pyproject.toml`, then from the repository root:

```bash
just publish-pypi-hprc
```

Use `just dry_run=true publish-pypi-hprc` to build without uploading. Requires maintainer permissions in the HPRC PyPI organization.

## Layout

```
processing/
  firm/           log archive (`firm.flight_data`) and CLI entry points
  firm/eskf_lab/  Python lab + native CMake replay of STM32 ESKF sources
```
