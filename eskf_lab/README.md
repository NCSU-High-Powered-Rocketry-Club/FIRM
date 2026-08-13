# FIRM ESKF desktop lab

This tool compiles the production STM32 error-state Kalman filter sources as a fast native
desktop executable, aligns decoded launch sensor CSVs, replays them through the filter, and
writes compressed Parquet results for analysis and plotting. It does not reimplement the filter
math in Python.

## Quick start

From the repository root:

```powershell
uv sync
uv run firm-eskf list
uv run firm-eskf run my-launch
uv run firm-eskf serve
```

With no dataset names, `prepare` and `run` process every complete directory under
`eskf_lab/datasets`:

```powershell
uv run firm-eskf run
```

The first run converts and aligns the large CSV files. Later runs reuse the cached Parquet and
binary input, so changing ESKF C code normally requires only an incremental native compile and
replay.

For the normal multi-launch workflow, run every dataset once and then use one tabbed dashboard:

```powershell
uv run firm-eskf run
uv run firm-eskf serve
```

## Dataset format

Each launch directory must contain the four decoded top-level files shown below. The raw `.FRM`
file and `Calibration/` directory are deliberately ignored.

```text
eskf_lab/datasets/my-launch/
  BMP581_data.csv
  ICM45686_data.csv
  MMC5983MA_data.csv
  ADXL371_data.csv
  LOG12.FRM                 # ignored
  Calibration/             # ignored
```

The metadata preamble is detected automatically. Column mappings and filenames live in
`eskf_lab/config/default.toml`; copy that profile and pass `--profile PATH` if a future decoder
uses different names. Unmapped columns are retained with a sensor prefix, so they can immediately
be selected in plots.

## Commands

```text
firm-eskf list
firm-eskf prepare [DATASET ...] [--force]
firm-eskf run [DATASET ...] [--force-prepare]
firm-eskf test
firm-eskf inspect RESULT
firm-eskf plot RESULT [--columns COLUMN ...] [--open]
firm-eskf compare RESULT RESULT [--columns COLUMN ...] [--open]
firm-eskf serve [RESULT] [--port 8050]
```

`RESULT` can be a `result.parquet` path, a run directory, or a dataset name. A dataset name selects
its latest run. `inspect` lists every available raw and filter column.

`serve` starts a dashboard whose column selector lazily reads only the requested Parquet fields.
With no `RESULT`, it opens every dataset's latest run in a single browser dashboard. Each dataset
has a top-level tab; tabs have a minimum width and wrap to additional rows on smaller screens. Pass
`RESULT` to serve only one dataset instead:

```powershell
uv run firm-eskf serve
uv run firm-eskf serve my-launch
```

`plot` writes a standalone HTML report for one run, while `compare` overlays the same selected
columns across two or more runs. Both use min/max downsampling so short spikes survive while the
browser receives a manageable number of points. When both are selected,
`raw_baro_altitude_m` and `eskf_position_z_m` are intentionally drawn together on the same
**Altitude (m)** chart for direct comparison.

## Replay model

The filter uses IMU, barometer, and magnetometer data. High-g accelerometer values are aligned and
retained for comparisons but are not passed to the current ESKF because
`STM32/Core/Src/tasks/filter_data_task.c` does not use that sensor.

Sensor files have independent timestamps. The magnetometer is the slowest required stream in FIRM
logs, so its samples form the update clock. Each replay row uses the most recent IMU, barometer,
and high-g sample at or before that timestamp, reproducing the firmware's shared-data snapshot
semantics. The first two seconds are accumulated for ESKF initialization.

The native target directly compiles:

- `error_state_kalman_filter.c`
- `eskf_functions.c`
- `eskf_config.c`
- `matrix_helper.c`

A small host shim supplies firmware settings without HAL or FreeRTOS. Results record the filter
source hash, Git revision and dirty state, dataset fingerprint, build path, timing, and descriptive
metrics. Pressure-derived altitude is useful for comparing smoothness but is explicitly not treated
as ground truth.

## Reading results and warnings

Each replay creates a timestamped directory under `eskf_lab/results/<dataset>/` containing:

- `result.parquet`: the aligned raw sensor fields, ESKF state, covariance diagonals, and diagnostics
- `metrics.json`: speed, apogee, smoothness, and finite-state summary
- `run.json`: source/configuration provenance for reproducing the run

`non-finite state values` means at least one ESKF state became `NaN` or `+/-Infinity`. This is a
filter numerical-stability warning, not a CSV parsing warning. Once it happens, later calculations
usually remain invalid. Use the last finite time region in the dashboard, covariance columns, and
the raw sensor traces to diagnose the configuration or model behavior that preceded it.

## Generated files

All datasets and generated artifacts are ignored by Git:

- `eskf_lab/cache`: aligned Parquet and compact replay input
- `eskf_lab/results`: timestamped result Parquet, metadata, metrics, and plots
- `eskf_lab/build`: native CMake build

Delete a dataset's cache or use `prepare --force` after changing its profile. Source file size and
modification time automatically invalidate stale data caches; CMake automatically rebuilds changed
filter sources.
