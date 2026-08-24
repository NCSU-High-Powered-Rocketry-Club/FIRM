"""Filesystem catalog and immutable ingestion for FIRM flight recordings."""

from __future__ import annotations

import hashlib
import re
import shutil
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from .fingerprint import build_fingerprint
from .formats import LogReader, normalize_hardware_version

SCHEMA_VERSION = 2
REQUIRED_PARQUET = ("barometer", "imu", "magnetometer", "high-g")
SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


def default_archive_root() -> Path:
    """Return the repository's configured flight-data root."""
    return Path(__file__).resolve().parents[3] / "flight_data"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_id(value: str, kind: str) -> str:
    if not SAFE_ID.fullmatch(value):
        raise ValueError(f"{kind} must use only letters, numbers, '.', '-', and '_'")
    return value


def read_yaml(path: Path) -> dict[str, Any]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return document


def write_yaml_atomic(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    temporary.replace(path)


@dataclass(frozen=True)
class Launch:
    """One launch that may contain several FIRM recordings."""

    archive: Archive
    launch_id: str
    path: Path

    @property
    def manifest_path(self) -> Path:
        return self.path / "launch.yaml"

    def recordings(self, *, current_only: bool = False) -> list[Recording]:
        root = self.path / "recordings"
        if not root.is_dir():
            return []
        result = [
            Recording(self, child.name, child)
            for child in root.iterdir()
            if child.is_dir() and (child / "recording.yaml").is_file()
        ]
        if current_only:
            result = [recording for recording in result if recording.is_current]
        return sorted(result, key=lambda item: item.recording_id.casefold())


@dataclass(frozen=True)
class Recording:
    """A flight log and its associated magnetometer-calibration log."""

    launch: Launch
    recording_id: str
    path: Path

    @property
    def dataset_id(self) -> str:
        return f"{self.launch.launch_id}/{self.recording_id}"

    @property
    def manifest_path(self) -> Path:
        return self.path / "recording.yaml"

    @property
    def manifest(self) -> dict[str, Any]:
        return read_yaml(self.manifest_path)

    @property
    def flight_original(self) -> Path:
        return self.path / "originals" / "flight.frm"

    @property
    def mag_cal_original(self) -> Path:
        return self.path / "originals" / "magnetometer-calibration.frm"

    @property
    def calibration_override(self) -> Path:
        return self.path / "overrides" / "calibration.yaml"

    @property
    def flight_derived(self) -> Path:
        return self.path / "derived" / "flight" / "current.frm"

    @property
    def mag_cal_derived(self) -> Path:
        return self.path / "derived" / "magnetometer-calibration" / "current.frm"

    @property
    def build_metadata_path(self) -> Path:
        return self.path / "derived" / "build.json"

    @property
    def decoded_dir(self) -> Path:
        return self.path / "decoded"

    def parquet_path(self, sensor: str) -> Path:
        return self.decoded_dir / f"{sensor}.parquet"

    @property
    def is_current(self) -> bool:
        manifest = self.manifest
        if manifest.get("build", {}).get("status") != "current":
            return False
        if manifest.get("build", {}).get("fingerprint") != build_fingerprint(self):
            return False
        source_paths = {
            "flight": self.flight_original,
            "magnetometer_calibration": self.mag_cal_original,
        }
        for role, source in manifest.get("sources", {}).items():
            path = source_paths.get(role)
            if (
                path is None
                or not path.is_file()
                or not isinstance(source, dict)
                or path.stat().st_size != source.get("size_bytes")
            ):
                return False
        required = [self.flight_derived]
        required.extend(self.parquet_path(name) for name in REQUIRED_PARQUET)
        if self.mag_cal_original.is_file():
            required.extend([self.mag_cal_derived, self.parquet_path("magnetometer-calibration")])
        return all(path.is_file() for path in required)

    def save_manifest(self, document: dict[str, Any]) -> None:
        write_yaml_atomic(self.manifest_path, document)

    @property
    def source_hardware(self) -> str:
        value = self.manifest.get("source_hardware")
        if not isinstance(value, str):
            raise ValueError(
                f"{self.dataset_id} has no source hardware; run "
                f"'firm-log hardware set {self.dataset_id} old|new'"
            )
        return normalize_hardware_version(value)

    def set_source_hardware(self, hardware: str) -> None:
        """Set authoritative source hardware and invalidate generated artifacts."""
        normalized = normalize_hardware_version(hardware)
        manifest = self.manifest
        manifest["schema_version"] = SCHEMA_VERSION
        manifest["source_hardware"] = normalized
        manifest["build"] = {"status": "stale", "reason": "source hardware changed"}
        self.save_manifest(manifest)


class Archive:
    """Manifest-backed flight archive."""

    def __init__(self, root: str | Path | None = None):
        self.root = Path(root).resolve() if root is not None else default_archive_root().resolve()
        self.config_path = self.root / "archive.yaml"
        if not self.config_path.is_file():
            raise FileNotFoundError(
                f"flight archive configuration does not exist: {self.config_path}"
            )
        config = read_yaml(self.config_path)
        self.launches_root = self.root / str(config.get("launches_directory", "launches"))

    def launches(self) -> list[Launch]:
        if not self.launches_root.is_dir():
            return []
        return sorted(
            [
                Launch(self, child.name, child)
                for child in self.launches_root.iterdir()
                if child.is_dir() and (child / "launch.yaml").is_file()
            ],
            key=lambda item: item.launch_id.casefold(),
        )

    def recordings(self, *, current_only: bool = False) -> list[Recording]:
        return [
            recording
            for launch in self.launches()
            for recording in launch.recordings(current_only=current_only)
        ]

    def resolve(self, value: str) -> Recording:
        normalized = value.replace("\\", "/").strip("/")
        if "/" in normalized:
            launch_id, recording_id = normalized.split("/", 1)
            path = self.launches_root / launch_id / "recordings" / recording_id
            if (path / "recording.yaml").is_file():
                return Recording(Launch(self, launch_id, path.parents[1]), recording_id, path)
            raise FileNotFoundError(f"managed recording does not exist: {value}")
        matches = [item for item in self.recordings() if item.recording_id == normalized]
        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            raise ValueError(f"recording ID {value!r} is ambiguous; use launch/recording")
        raise FileNotFoundError(f"managed recording does not exist: {value}")

    def ingest(
        self,
        launch_id: str,
        recording_id: str,
        flight: str | Path,
        mag_cal: str | Path | None = None,
        *,
        hardware: str,
    ) -> Recording:
        """Copy immutable acquisition logs into a new recording."""
        launch_id = _validate_id(launch_id, "launch ID")
        recording_id = _validate_id(recording_id, "recording ID")
        hardware = normalize_hardware_version(hardware)
        flight = Path(flight).resolve()
        mag_cal_path = Path(mag_cal).resolve() if mag_cal is not None else None
        mag_cal_source_name = mag_cal_path.name if mag_cal_path is not None else ""
        if not flight.is_file():
            raise FileNotFoundError(f"flight log does not exist: {flight}")
        if mag_cal_path is not None and not mag_cal_path.is_file():
            raise FileNotFoundError(f"magnetometer-calibration log does not exist: {mag_cal_path}")

        # Reject unsupported files before changing the archive.
        flight_reader = LogReader(flight)
        mag_reader = LogReader(mag_cal_path) if mag_cal_path is not None else None
        launch_path = self.launches_root / launch_id
        recording_path = launch_path / "recordings" / recording_id
        if recording_path.exists():
            raise FileExistsError(f"recording already exists: {launch_id}/{recording_id}")
        originals = recording_path / "originals"
        originals.mkdir(parents=True)
        try:
            flight_destination = originals / "flight.frm"
            shutil.copyfile(flight, flight_destination)
            mag_destination: Path | None = None
            if mag_cal_path is not None:
                mag_destination = originals / "magnetometer-calibration.frm"
                shutil.copyfile(mag_cal_path, mag_destination)
            now = datetime.now(tz=timezone.utc).isoformat()
            sources: dict[str, Any] = {
                "flight": {
                    "path": "originals/flight.frm",
                    "source_filename": flight.name,
                    "sha256": sha256_file(flight_destination),
                    "size_bytes": flight_destination.stat().st_size,
                    "log_version": flight_reader.header.version,
                }
            }
            if mag_destination is not None and mag_reader is not None:
                sources["magnetometer_calibration"] = {
                    "path": "originals/magnetometer-calibration.frm",
                    "source_filename": mag_cal_source_name,
                    "sha256": sha256_file(mag_destination),
                    "size_bytes": mag_destination.stat().st_size,
                    "log_version": mag_reader.header.version,
                }
            manifest = {
                "schema_version": SCHEMA_VERSION,
                "launch_id": launch_id,
                "recording_id": recording_id,
                "created_utc": now,
                "source_hardware": hardware,
                "sources": sources,
                "trim_window": None,
                "trim_proposals": {},
                "calibration_override": None,
                "build": {"status": "not_built"},
            }
            write_yaml_atomic(recording_path / "recording.yaml", manifest)
            if not (launch_path / "launch.yaml").is_file():
                write_yaml_atomic(
                    launch_path / "launch.yaml",
                    {"schema_version": SCHEMA_VERSION, "launch_id": launch_id, "created_utc": now},
                )
        except Exception:
            # This path was just created by this call, so removing it cannot affect existing data.
            shutil.rmtree(recording_path, ignore_errors=True)
            raise
        return self.resolve(f"{launch_id}/{recording_id}")
