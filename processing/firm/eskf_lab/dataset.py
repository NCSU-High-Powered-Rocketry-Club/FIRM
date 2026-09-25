"""Fast preparation of manager-built FIRM flight recordings."""

from __future__ import annotations

import hashlib
import json
import math
import re
import struct
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np
import polars as pl

from firm.flight_data import Archive, Recording
from firm.flight_data.archive import sha256_file
from firm.flight_data.formats import TARGET_HARDWARE, LogReader

if TYPE_CHECKING:
    from pathlib import Path


CACHE_FORMAT_VERSION = 4
INPUT_MAGIC = b"FIRMIN01"
INPUT_VERSION = 1
INITIALIZATION_SECONDS = 2.0
REPLAY_COLUMNS = (
    "pressure_pa",
    "temperature_c",
    "imu_accel_x_g",
    "imu_accel_y_g",
    "imu_accel_z_g",
    "imu_gyro_x_dps",
    "imu_gyro_y_dps",
    "imu_gyro_z_dps",
    "mag_x_ut",
    "mag_y_ut",
    "mag_z_ut",
    "high_g_accel_x_g",
    "high_g_accel_y_g",
    "high_g_accel_z_g",
)
REQUIRED_COLUMNS = tuple(column for column in REPLAY_COLUMNS if not column.startswith("high_g_"))
INPUT_RECORD_DTYPE = np.dtype(
    [("timestamp", "<f8"), ("values", "<f4", (len(REPLAY_COLUMNS),))], align=False
)
SENSOR_NAMES = ("barometer", "imu", "magnetometer", "high_g")
CANONICAL_REPLAY_FIRMWARE_VERSION = "v2.0.0"


@dataclass(frozen=True)
class PreparedDataset:
    """Paths and metadata for a prepared manager recording."""

    dataset_id: str
    dataset_path: Path
    cache_path: Path
    parquet_path: Path
    replay_path: Path
    metadata_path: Path
    metadata: dict[str, Any]


def safe_name(value: str) -> str:
    """Return a filesystem-friendly dataset name."""
    name = re.sub(r"[^A-Za-z0-9_.-]+", "-", value.strip()).strip("-.")
    return name or "dataset"


def discover_datasets(flight_data_root: Path, _profile: object | None = None) -> list[Recording]:
    """Return only current recordings from the flight-data manager."""
    return Archive(flight_data_root).recordings(current_only=True)


def resolve_dataset(
    value: str, flight_data_root: Path, _profile: object | None = None
) -> Recording:
    """Resolve and validate a manager recording."""
    recording = Archive(flight_data_root).resolve(value)
    if not recording.is_current:
        raise ValueError(
            f"managed recording {recording.dataset_id} is not current; run "
            f"'firm-log build {recording.dataset_id}'"
        )
    return recording


def _source_fingerprint(recording: Recording) -> tuple[str, list[dict[str, Any]]]:
    manifest = recording.manifest
    manager_fingerprint = manifest.get("build", {}).get("fingerprint")
    if not isinstance(manager_fingerprint, str):
        raise TypeError(f"{recording.dataset_id} has no current manager build fingerprint")
    digest = hashlib.sha256()
    digest.update(f"eskf-cache-v{CACHE_FORMAT_VERSION}\0".encode())
    digest.update(manager_fingerprint.encode())
    sources: list[dict[str, Any]] = []
    for sensor in SENSOR_NAMES:
        filename = "high-g" if sensor == "high_g" else sensor
        path = recording.parquet_path(filename)
        item = {
            "sensor": sensor,
            "filename": path.name,
            "size_bytes": path.stat().st_size,
            "sha256": sha256_file(path),
        }
        sources.append(item)
        digest.update(json.dumps(item, sort_keys=True).encode())
    return digest.hexdigest(), sources


def _scan_sensor(recording: Recording, sensor: str) -> pl.LazyFrame:
    filename = "high-g" if sensor == "high_g" else sensor
    path = recording.parquet_path(filename)
    frame = pl.scan_parquet(path)
    source_columns = frame.collect_schema().names()
    if "timestamp_s" not in source_columns:
        raise ValueError(f"{path.name} has no 'timestamp_s' column")
    sensor_prefixes: str | tuple[str, ...] = {
        "barometer": ("pressure_", "temperature_"),
        "imu": "imu_",
        "magnetometer": "mag_",
        "high_g": "high_g_",
    }[sensor]
    missing = sorted(
        column
        for column in REPLAY_COLUMNS
        if column.startswith(sensor_prefixes) and column not in source_columns
    )
    if missing:
        raise ValueError(f"{path.name} is missing canonical columns: {', '.join(missing)}")
    rename: dict[str, str] = {"timestamp_s": "timestamp"}
    for column in source_columns:
        if column == "timestamp_s" or column in REPLAY_COLUMNS:
            continue
        rename[column] = f"{sensor}__{column}"
    return (
        frame.rename(rename)
        .with_columns(pl.col("timestamp").cast(pl.Float64))
        .sort("timestamp")
        .unique(subset=["timestamp"], keep="last", maintain_order=True)
    )


def _aligned_frame(recording: Recording) -> pl.LazyFrame:
    frames = {name: _scan_sensor(recording, name) for name in SENSOR_NAMES}
    aligned = frames["magnetometer"]
    for name in ("imu", "barometer"):
        aligned = aligned.join_asof(frames[name], on="timestamp", strategy="backward")
    high_g_empty = frames["high_g"].select(pl.len()).collect(engine="streaming").item() == 0
    if high_g_empty:
        aligned = aligned.with_columns(
            [
                pl.lit(float("nan")).alias(name)
                for name in REPLAY_COLUMNS
                if name.startswith("high_g_")
            ]
        )
    else:
        aligned = aligned.join_asof(frames["high_g"], on="timestamp", strategy="backward")
    return aligned.drop_nulls(list(REQUIRED_COLUMNS)).sort("timestamp")


def _write_replay_binary(frame: pl.DataFrame, path: Path, firmware_version: str) -> None:
    columns: list[np.ndarray] = []
    for name in REPLAY_COLUMNS:
        values = frame.get_column(name).cast(pl.Float32).fill_null(float("nan")).to_numpy()
        columns.append(np.asarray(values, dtype="<f4"))
    records = np.empty(frame.height, dtype=INPUT_RECORD_DTYPE)
    records["timestamp"] = frame.get_column("timestamp").to_numpy()
    records["values"] = np.column_stack(columns)
    firmware = firmware_version.encode("ascii", errors="replace")[:8].ljust(8, b"\0")
    header = struct.pack(
        "<8sIIQd8s",
        INPUT_MAGIC,
        INPUT_VERSION,
        INPUT_RECORD_DTYPE.itemsize,
        frame.height,
        INITIALIZATION_SECONDS,
        firmware,
    )
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as handle:
        handle.write(header)
        records.tofile(handle)
    temporary.replace(path)


def prepare_dataset(
    recording: Recording,
    _profile: object,
    cache_root: Path,
    *,
    force: bool = False,
) -> PreparedDataset:
    """Align and cache one current manager recording."""
    if not isinstance(recording, Recording):
        raise TypeError("ESKF datasets must be flight-data manager recordings")
    if not recording.is_current:
        raise ValueError(f"managed recording is not current: {recording.dataset_id}")
    fingerprint, sources = _source_fingerprint(recording)
    cache_path = cache_root.resolve() / safe_name(recording.dataset_id) / fingerprint[:16]
    parquet_path = cache_path / "aligned.parquet"
    replay_path = cache_path / "replay.bin"
    metadata_path = cache_path / "prepared.json"
    if not force and parquet_path.is_file() and replay_path.is_file() and metadata_path.is_file():
        return PreparedDataset(
            recording.dataset_id,
            recording.path,
            cache_path,
            parquet_path,
            replay_path,
            metadata_path,
            json.loads(metadata_path.read_text()),
        )

    cache_path.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    frame = _aligned_frame(recording).collect(engine="streaming")
    if frame.is_empty():
        raise ValueError(f"{recording.dataset_id} has no aligned required sensor values")
    source_firmware = LogReader(recording.flight_derived).header.firmware_version
    firmware = CANONICAL_REPLAY_FIRMWARE_VERSION
    parquet_temporary = parquet_path.with_suffix(".parquet.tmp")
    frame.write_parquet(parquet_temporary, compression="zstd", statistics=True)
    parquet_temporary.replace(parquet_path)
    _write_replay_binary(frame, replay_path, firmware)

    duration = float(frame["timestamp"][-1] - frame["timestamp"][0]) if frame.height > 1 else 0.0
    metadata: dict[str, Any] = {
        "cache_format_version": CACHE_FORMAT_VERSION,
        "dataset": recording.dataset_id,
        "dataset_path": str(recording.path),
        "fingerprint": fingerprint,
        "manager_build_fingerprint": recording.manifest["build"]["fingerprint"],
        "calibration_applied": True,
        "firmware_version": firmware,
        "source_firmware_version": source_firmware,
        "source_hardware": recording.source_hardware,
        "target_hardware": TARGET_HARDWARE,
        "rows": frame.height,
        "start_time_seconds": float(frame["timestamp"][0]),
        "end_time_seconds": float(frame["timestamp"][-1]),
        "duration_seconds": duration,
        "columns": frame.columns,
        "sources": sources,
        "prepare_seconds": time.perf_counter() - started,
    }
    metadata_path.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return PreparedDataset(
        recording.dataset_id,
        recording.path,
        cache_path,
        parquet_path,
        replay_path,
        metadata_path,
        metadata,
    )


def human_size(size: int) -> str:
    """Format a byte count for CLI output."""
    value = float(size)
    for suffix in ("B", "KiB", "MiB", "GiB", "TiB"):
        if abs(value) < 1024.0 or suffix == "TiB":
            return f"{value:.1f} {suffix}"
        value /= 1024.0
    return f"{math.nan} B"
