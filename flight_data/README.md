# FIRM flight data

This directory is the catalog root for the FIRM flight-data manager.

Each launch can contain several recordings (for example, airbrakes, grave, and zombie). A recording
contains an immutable flight log, an optional immutable magnetometer-calibration log, a manifest,
and rebuildable derived artifacts.

```text
flight_data/launches/<launch>/recordings/<recording>/
  recording.yaml
  originals/flight.frm
  originals/magnetometer-calibration.frm
  overrides/calibration.yaml
  derived/flight/current.frm
  derived/magnetometer-calibration/current.frm
  derived/effective-calibration.yaml
  derived/build.json
  decoded/barometer.parquet
  decoded/imu.parquet
  decoded/magnetometer.parquet
  decoded/high-g.parquet
  decoded/magnetometer-calibration.parquet
```

Start by ingesting a pair of logs, then build their current artifacts:

```powershell
uv run firm-log ingest --launch jackpot-4 --recording airbrakes `
  --flight C:\logs\LOG42.FRM --mag-cal C:\logs\LOG40.FRM --hardware new
uv run firm-log build jackpot-4/airbrakes
uv run firm-eskf list
```

Originals are copied and SHA-256 hashed. Manager commands never modify them. Calibration overrides,
trim windows, migrated logs, and decoded data are all reproducible from `recording.yaml`.

Every recording declares `--hardware old|new` at ingestion. Migration rotates old-hardware IMU
accelerometer and gyroscope bins into the new-hardware sensor frame and transforms their calibration
offsets and matrices with the same change of basis. Barometer and magnetometer data are unchanged;
old recordings may legitimately produce an empty high-g Parquet file. For recordings ingested
before this field existed, use `firm-log hardware set LAUNCH/RECORDING old|new` before building.
Calibration overrides are written in the recording's original sensor frame; the build transforms
old-hardware accel and gyro overrides along with the calibration stored in the original header.

Use `firm-log inspect`, `validate`, and `status` to audit recordings. Calibration changes use
`firm-log calibration set`; explicit or reviewed phase trim windows use `firm-log trim`; `rebuild
--all` refreshes every stale recording. CSV remains an explicit export format rather than an
authoritative artifact.

Git tracks catalog manifests and documentation, but ignores acquired logs, derived logs, Parquet,
and local indexes. A team storage or synchronization system can therefore mirror the complete
`flight_data` tree without putting large binaries in normal Git history; `archive.yaml`,
`launch.yaml`, and `recording.yaml` remain the authoritative, rebuildable catalog.
