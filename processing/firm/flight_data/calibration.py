"""Dataset-specific calibration override handling."""

from __future__ import annotations

from pathlib import Path

import yaml

from .archive import Recording, sha256_file, write_yaml_atomic
from .formats import LogReader
from .models import CalibrationOverride, EffectiveCalibration


def load_override(path: Path) -> CalibrationOverride:
    if not path.is_file():
        return CalibrationOverride()
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if document is not None and not isinstance(document, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return CalibrationOverride.from_mapping(document)


def effective_calibration(recording: Recording) -> EffectiveCalibration:
    base = LogReader(recording.flight_original).header.calibration
    return load_override(recording.calibration_override).apply(base)


def set_calibration_override(recording: Recording, source: str | Path) -> CalibrationOverride:
    source = Path(source).resolve()
    if not source.is_file():
        raise FileNotFoundError(f"calibration override does not exist: {source}")
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("calibration override must contain a YAML mapping")
    override = CalibrationOverride.from_mapping(document)
    normalized = {
        "schema_version": 1,
        "calibration": {
            sensor: {
                field_name: (
                    [list(values[index : index + 3]) for index in range(0, 9, 3)]
                    if field_name == "matrix"
                    else list(values)
                )
                for field_name, values in fields.items()
            }
            for sensor, fields in override.values.items()
        },
    }
    write_yaml_atomic(recording.calibration_override, normalized)
    manifest = recording.manifest
    manifest["calibration_override"] = {
        "path": "overrides/calibration.yaml",
        "sha256": sha256_file(recording.calibration_override),
    }
    manifest["build"] = {"status": "stale", "reason": "calibration override changed"}
    recording.save_manifest(manifest)
    return override
