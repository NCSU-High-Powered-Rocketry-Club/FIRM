"""Streaming native-rate Parquet generation and optional CSV export."""

from __future__ import annotations

import json
import tempfile
from pathlib import Path
from typing import Any

import polars as pl

from .formats import LogReader, decode_packet
from .models import EffectiveCalibration

CHUNK_ROWS = 100_000
SENSOR_FILENAMES = {
    "barometer": "barometer.parquet",
    "imu": "imu.parquet",
    "magnetometer": "magnetometer.parquet",
    "high_g": "high-g.parquet",
}

COMMON_SCHEMA = {
    "packet_index": pl.UInt64,
    "timestamp_ticks": pl.UInt64,
    "timestamp_s": pl.Float64,
    "trimmed_timestamp_s": pl.Float64,
}
SENSOR_SCHEMA: dict[str, dict[str, Any]] = {
    "barometer": {
        **COMMON_SCHEMA,
        "raw_temperature": pl.UInt32,
        "raw_pressure": pl.UInt32,
        "uncalibrated_temperature_c": pl.Float64,
        "uncalibrated_pressure_pa": pl.Float64,
        "temperature_c": pl.Float64,
        "pressure_pa": pl.Float64,
    },
    "imu": {
        **COMMON_SCHEMA,
        **{f"raw_accel_{axis}": pl.Int32 for axis in "xyz"},
        **{f"raw_gyro_{axis}": pl.Int32 for axis in "xyz"},
        **{f"uncalibrated_accel_{axis}_g": pl.Float64 for axis in "xyz"},
        **{f"uncalibrated_gyro_{axis}_dps": pl.Float64 for axis in "xyz"},
        **{f"imu_accel_{axis}_g": pl.Float64 for axis in "xyz"},
        **{f"imu_gyro_{axis}_dps": pl.Float64 for axis in "xyz"},
    },
    "magnetometer": {
        **COMMON_SCHEMA,
        **{f"raw_mag_{axis}": pl.UInt32 for axis in "xyz"},
        **{f"uncalibrated_mag_{axis}_ut": pl.Float64 for axis in "xyz"},
        **{f"mag_{axis}_ut": pl.Float64 for axis in "xyz"},
    },
    "high_g": {
        **COMMON_SCHEMA,
        **{f"raw_accel_{axis}": pl.Int32 for axis in "xyz"},
        **{f"uncalibrated_accel_{axis}_g": pl.Float64 for axis in "xyz"},
        **{f"high_g_accel_{axis}_g": pl.Float64 for axis in "xyz"},
    },
}


def _column_units(sensor: str) -> dict[str, str]:
    units = {
        "packet_index": "1",
        "timestamp_ticks": "168 MHz clock ticks",
        "timestamp_s": "s",
        "trimmed_timestamp_s": "s",
    }
    for column in SENSOR_SCHEMA[sensor]:
        if column.startswith("raw_"):
            units[column] = "sensor counts"
        elif column.endswith("_c"):
            units[column] = "degC"
        elif column.endswith("_pa"):
            units[column] = "Pa"
        elif column.endswith("_dps"):
            units[column] = "deg/s"
        elif column.endswith("_ut"):
            units[column] = "uT"
        elif column.endswith("_g"):
            units[column] = "g"
    return units


def _frame(rows: list[dict[str, Any]], sensor: str) -> pl.DataFrame:
    schema = SENSOR_SCHEMA[sensor]
    if not rows:
        return pl.DataFrame(schema=schema)
    return pl.DataFrame(rows, schema=schema)


def decode_to_parquet(
    log_path: Path,
    output_dir: Path,
    calibration: EffectiveCalibration,
    *,
    trimmed_start_s: float = 0.0,
    metadata: dict[str, str] | None = None,
    magnetometer_calibration: bool = False,
    time_origin_ticks: int | None = None,
    first_unwrapped_ticks: int | None = None,
) -> tuple[dict[str, Path], dict[str, int]]:
    """Decode one migrated log into bounded-memory, native-rate Parquet files."""
    output_dir.mkdir(parents=True, exist_ok=True)
    selected = ("magnetometer",) if magnetometer_calibration else tuple(SENSOR_FILENAMES)
    rows: dict[str, list[dict[str, Any]]] = {sensor: [] for sensor in selected}
    part_paths: dict[str, list[Path]] = {sensor: [] for sensor in selected}
    counts = dict.fromkeys(selected, 0)
    with tempfile.TemporaryDirectory(prefix="firm-decode-", dir=output_dir) as temporary_name:
        temporary_root = Path(temporary_name)

        def flush(sensor: str) -> None:
            if not rows[sensor]:
                return
            part = temporary_root / f"{sensor}-{len(part_paths[sensor]):05d}.parquet"
            _frame(rows[sensor], sensor).write_parquet(part, compression="zstd", statistics=True)
            part_paths[sensor].append(part)
            rows[sensor].clear()

        reader = LogReader(
            log_path,
            time_origin_ticks=time_origin_ticks,
            first_unwrapped_ticks=first_unwrapped_ticks,
        )
        for packet in reader.iter_packets():
            if packet.sensor not in rows:
                continue
            decoded = decode_packet(packet, calibration)
            decoded["trimmed_timestamp_s"] = packet.timestamp_s - trimmed_start_s
            rows[packet.sensor].append(decoded)
            counts[packet.sensor] += 1
            if len(rows[packet.sensor]) >= CHUNK_ROWS:
                flush(packet.sensor)
        for sensor in selected:
            flush(sensor)

        written: dict[str, Path] = {}
        for sensor in selected:
            parquet_metadata = {
                "firm.flight_data": json.dumps(
                    metadata or {}, sort_keys=True, separators=(",", ":")
                ),
                "firm.units": json.dumps(
                    _column_units(sensor), sort_keys=True, separators=(",", ":")
                ),
            }
            filename = (
                "magnetometer-calibration.parquet"
                if magnetometer_calibration
                else SENSOR_FILENAMES[sensor]
            )
            final = output_dir / filename
            temporary = final.with_suffix(final.suffix + ".tmp")
            if part_paths[sensor]:
                pl.concat([pl.scan_parquet(path) for path in part_paths[sensor]]).sink_parquet(
                    temporary,
                    compression="zstd",
                    statistics=True,
                    metadata=parquet_metadata,
                )
            else:
                _frame([], sensor).write_parquet(
                    temporary,
                    compression="zstd",
                    statistics=True,
                    metadata=parquet_metadata,
                )
            temporary.replace(final)
            written[sensor] = final
    return written, counts


def validate_parquet_outputs(paths: dict[str, Path], expected_counts: dict[str, int]) -> None:
    """Validate generated schemas, metadata, and row counts before promotion."""
    for sensor, path in paths.items():
        schema = pl.scan_parquet(path).collect_schema()
        missing = set(SENSOR_SCHEMA[sensor]) - set(schema.names())
        if missing:
            raise ValueError(f"{path.name} is missing columns: {', '.join(sorted(missing))}")
        metadata = pl.read_parquet_metadata(path)
        if "firm.flight_data" not in metadata or "firm.units" not in metadata:
            raise ValueError(f"{path.name} is missing FIRM provenance or unit metadata")
        row_count = pl.scan_parquet(path).select(pl.len()).collect(engine="streaming").item()
        if row_count != expected_counts[sensor]:
            raise ValueError(
                f"{path.name} has {row_count} rows; expected {expected_counts[sensor]}"
            )


def export_csv(decoded_dir: Path, output_dir: Path) -> list[Path]:
    """Export canonical Parquet artifacts as explicitly requested CSV files."""
    output_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    for parquet in sorted(decoded_dir.glob("*.parquet")):
        output = output_dir / f"{parquet.stem}.csv"
        pl.scan_parquet(parquet).sink_csv(output)
        written.append(output)
    if not written:
        raise FileNotFoundError(f"no decoded Parquet files exist under {decoded_dir}")
    return written
