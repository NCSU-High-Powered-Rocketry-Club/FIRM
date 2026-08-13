"""Dataset profile loading and validation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import tomllib

if TYPE_CHECKING:
    from pathlib import Path

SENSOR_NAMES = ("barometer", "imu", "magnetometer", "high_g")


@dataclass(frozen=True)
class SensorProfile:
    """CSV filename and column mapping for one sensor."""

    name: str
    filename: str
    timestamp: str
    columns: dict[str, str]


@dataclass(frozen=True)
class DatasetProfile:
    """Mapping from a decoded launch folder to canonical column names."""

    path: Path
    version: int
    sensors: dict[str, SensorProfile]
    raw_bytes: bytes


def _mapping(table: dict[str, Any], name: str) -> dict[str, str]:
    value = table.get(name)
    if not isinstance(value, dict) or not all(
        isinstance(key, str) and isinstance(item, str) for key, item in value.items()
    ):
        raise ValueError(f"profile field {name!r} must be a string-to-string table")
    return value


def load_profile(path: Path) -> DatasetProfile:
    """Load a TOML dataset profile."""
    path = path.resolve()
    raw = path.read_bytes()
    document = tomllib.loads(raw.decode("utf-8"))
    files = _mapping(document, "files")
    sensor_tables = document.get("sensors")
    if not isinstance(sensor_tables, dict):
        raise ValueError("profile must contain a [sensors] table")

    sensors: dict[str, SensorProfile] = {}
    for name in SENSOR_NAMES:
        sensor = sensor_tables.get(name)
        if not isinstance(sensor, dict):
            raise ValueError(f"profile is missing [sensors.{name}]")
        filename = files.get(name)
        timestamp = sensor.get("timestamp")
        columns = sensor.get("columns")
        if not isinstance(filename, str) or not isinstance(timestamp, str):
            raise ValueError(f"profile has an invalid filename or timestamp for {name}")
        if not isinstance(columns, dict) or not all(
            isinstance(key, str) and isinstance(value, str) for key, value in columns.items()
        ):
            raise ValueError(f"profile has an invalid column table for {name}")
        sensors[name] = SensorProfile(name, filename, timestamp, columns)

    version = document.get("version", 1)
    if not isinstance(version, int):
        raise ValueError("profile version must be an integer")
    return DatasetProfile(path, version, sensors, raw)
