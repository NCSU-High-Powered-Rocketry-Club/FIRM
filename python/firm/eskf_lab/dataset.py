"""Fast preparation of decoded FIRM launch datasets."""

from __future__ import annotations

import hashlib
import json
import math
import re
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
import polars as pl

if TYPE_CHECKING:
    from .profile import DatasetProfile, SensorProfile

CACHE_FORMAT_VERSION = 1
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


@dataclass(frozen=True)
class PreparedDataset:
    """Paths and metadata for a prepared launch dataset."""

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


def discover_datasets(datasets_dir: Path, profile: DatasetProfile) -> list[Path]:
    """Find direct child directories containing all four required sensor CSVs."""
    datasets_dir = datasets_dir.resolve()
    if not datasets_dir.is_dir():
        return []
    filenames = {sensor.filename for sensor in profile.sensors.values()}
    return sorted(
        (
            child
            for child in datasets_dir.iterdir()
            if child.is_dir() and all((child / filename).is_file() for filename in filenames)
        ),
        key=lambda item: item.name.casefold(),
    )


def resolve_dataset(value: str, datasets_dir: Path, profile: DatasetProfile) -> Path:
    """Resolve a dataset name or path and validate its expected files."""
    supplied = Path(value).expanduser()
    path = supplied if supplied.is_dir() else datasets_dir / value
    path = path.resolve()
    missing = [
        sensor.filename
        for sensor in profile.sensors.values()
        if not (path / sensor.filename).is_file()
    ]
    if missing:
        raise FileNotFoundError(f"{path} is missing required sensor files: {', '.join(missing)}")
    return path


def _header_and_metadata(path: Path) -> tuple[int, dict[str, str]]:
    metadata: dict[str, str] = {}
    with path.open("r", encoding="utf-8-sig", errors="replace", newline="") as handle:
        for line_number, line in enumerate(handle):
            stripped = line.strip()
            first = stripped.split(",", 1)[0].strip().lower()
            if first == "timestamp":
                return line_number, metadata
            if stripped and "," in stripped:
                key, value = stripped.split(",", 1)
                metadata[key.strip().rstrip(":")] = value.strip().rstrip(",")
            if line_number >= 128:
                break
    raise ValueError(f"could not find the timestamp header in {path}")


def _source_fingerprint(dataset: Path, profile: DatasetProfile) -> tuple[str, list[dict[str, Any]]]:
    digest = hashlib.sha256()
    digest.update(f"eskf-cache-v{CACHE_FORMAT_VERSION}\0".encode())
    digest.update(profile.raw_bytes)
    sources: list[dict[str, Any]] = []
    for name in sorted(profile.sensors):
        path = dataset / profile.sensors[name].filename
        stat = path.stat()
        item = {
            "sensor": name,
            "filename": path.name,
            "size_bytes": stat.st_size,
            "modified_ns": stat.st_mtime_ns,
        }
        sources.append(item)
        digest.update(json.dumps(item, sort_keys=True).encode())
    return digest.hexdigest(), sources


def _scan_sensor(path: Path, sensor: SensorProfile) -> tuple[pl.LazyFrame, dict[str, str]]:
    header_line, metadata = _header_and_metadata(path)
    frame = pl.scan_csv(
        path,
        skip_rows=header_line,
        has_header=True,
        infer_schema_length=10_000,
        rechunk=False,
        low_memory=False,
    )
    source_columns = frame.collect_schema().names()
    if sensor.timestamp not in source_columns:
        raise ValueError(f"{path.name} has no {sensor.timestamp!r} timestamp column")
    missing = sorted(set(sensor.columns) - set(source_columns))
    if missing:
        raise ValueError(f"{path.name} is missing configured columns: {', '.join(missing)}")

    rename: dict[str, str] = {sensor.timestamp: "timestamp"}
    rename.update(sensor.columns)
    for column in source_columns:
        if column not in rename:
            rename[column] = f"{sensor.name}__{column}"
    frame = (
        frame.rename(rename)
        .with_columns(pl.col("timestamp").cast(pl.Float64))
        .sort("timestamp")
        .unique(subset=["timestamp"], keep="last", maintain_order=True)
    )
    return frame, metadata


def _aligned_frame(dataset: Path, profile: DatasetProfile) -> tuple[pl.LazyFrame, dict[str, str]]:
    frames: dict[str, pl.LazyFrame] = {}
    combined_metadata: dict[str, str] = {}
    for name, sensor in profile.sensors.items():
        frame, metadata = _scan_sensor(dataset / sensor.filename, sensor)
        frames[name] = frame
        if name == "imu":
            combined_metadata = metadata

    # A filter update needs fresh IMU, barometer, and magnetometer bits. The magnetometer is
    # the slowest required stream in FIRM logs, so its timestamps are the replay clock and each
    # update receives the latest values from the other sensors, matching the shared snapshot.
    aligned = frames["magnetometer"]
    for name in ("imu", "barometer", "high_g"):
        aligned = aligned.join_asof(frames[name], on="timestamp", strategy="backward")
    aligned = aligned.drop_nulls(list(REQUIRED_COLUMNS)).sort("timestamp")
    return aligned, combined_metadata


def _firmware_version(metadata: dict[str, str]) -> str:
    value = metadata.get("FIRM version", "v2.0.0").strip()
    return value[:7] if value else "v2.0.0"


def _write_replay_binary(frame: pl.DataFrame, path: Path, firmware_version: str) -> None:
    columns: list[np.ndarray] = []
    for name in REPLAY_COLUMNS:
        if name in frame.columns:
            values = frame.get_column(name).cast(pl.Float32).fill_null(float("nan")).to_numpy()
        else:
            values = np.full(frame.height, np.nan, dtype=np.float32)
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
    dataset: Path,
    profile: DatasetProfile,
    cache_root: Path,
    *,
    force: bool = False,
) -> PreparedDataset:
    """Align and cache one launch dataset as Parquet and native replay records."""
    dataset = dataset.resolve()
    fingerprint, sources = _source_fingerprint(dataset, profile)
    cache_path = cache_root.resolve() / safe_name(dataset.name) / fingerprint[:16]
    parquet_path = cache_path / "aligned.parquet"
    replay_path = cache_path / "replay.bin"
    metadata_path = cache_path / "prepared.json"
    if not force and parquet_path.is_file() and replay_path.is_file() and metadata_path.is_file():
        return PreparedDataset(
            dataset,
            cache_path,
            parquet_path,
            replay_path,
            metadata_path,
            json.loads(metadata_path.read_text()),
        )

    cache_path.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    lazy_frame, csv_metadata = _aligned_frame(dataset, profile)
    frame = lazy_frame.collect(engine="streaming")
    if frame.is_empty():
        raise ValueError(f"{dataset} has no rows with all required ESKF sensor values")
    firmware = _firmware_version(csv_metadata)

    parquet_temporary = parquet_path.with_suffix(".parquet.tmp")
    frame.write_parquet(parquet_temporary, compression="zstd", statistics=True)
    parquet_temporary.replace(parquet_path)
    _write_replay_binary(frame, replay_path, firmware)

    duration = float(frame["timestamp"][-1] - frame["timestamp"][0]) if frame.height > 1 else 0.0
    metadata: dict[str, Any] = {
        "cache_format_version": CACHE_FORMAT_VERSION,
        "dataset": dataset.name,
        "dataset_path": str(dataset),
        "fingerprint": fingerprint,
        "profile": str(profile.path),
        "profile_version": profile.version,
        "firmware_version": firmware,
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
    return PreparedDataset(dataset, cache_path, parquet_path, replay_path, metadata_path, metadata)


def human_size(size: int) -> str:
    """Format a byte count for CLI output."""
    value = float(size)
    for suffix in ("B", "KiB", "MiB", "GiB", "TiB"):
        if abs(value) < 1024.0 or suffix == "TiB":
            return f"{value:.1f} {suffix}"
        value /= 1024.0
    return f"{math.nan} B"
