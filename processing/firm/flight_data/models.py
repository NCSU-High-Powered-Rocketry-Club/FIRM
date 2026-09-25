"""Public data models for the FIRM flight archive."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, cast

SENSOR_CALIBRATIONS = ("accel", "gyro", "mag", "high_g")


def _finite_tuple(value: Any, length: int, name: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must contain exactly {length} values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain only finite values")
    return result


@dataclass(frozen=True)
class CalibrationValues:
    """Three offsets and a row-major 3x3 calibration matrix."""

    offset: tuple[float, float, float]
    matrix: tuple[float, float, float, float, float, float, float, float, float]

    @classmethod
    def identity(cls) -> CalibrationValues:
        return cls((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))

    @classmethod
    def from_mapping(cls, value: dict[str, Any], name: str) -> CalibrationValues:
        offset = _finite_tuple(value.get("offset"), 3, f"{name}.offset")
        matrix_value = value.get("matrix", value.get("scale"))
        if (
            isinstance(matrix_value, list)
            and len(matrix_value) == 3
            and all(isinstance(row, list) for row in matrix_value)
        ):
            matrix_value = [item for row in matrix_value for item in row]
        matrix = _finite_tuple(matrix_value, 9, f"{name}.matrix")
        return cls(
            cast("tuple[float, float, float]", offset),
            cast("tuple[float, float, float, float, float, float, float, float, float]", matrix),
        )

    def to_mapping(self) -> dict[str, Any]:
        return {
            "offset": list(self.offset),
            "matrix": [list(self.matrix[index : index + 3]) for index in range(0, 9, 3)],
        }


@dataclass(frozen=True)
class CalibrationOverride:
    """Optional per-field calibration overrides for one recording."""

    values: dict[str, dict[str, tuple[float, ...]]] = field(default_factory=dict)

    @classmethod
    def from_mapping(cls, document: dict[str, Any] | None) -> CalibrationOverride:
        if not document:
            return cls()
        source = document.get("calibration", document)
        if not isinstance(source, dict):
            raise ValueError("calibration override must be a mapping")
        result: dict[str, dict[str, tuple[float, ...]]] = {}
        aliases = {
            "accel": "accel",
            "gyro": "gyro",
            "mag": "mag",
            "magnetometer": "mag",
            "high_g": "high_g",
        }
        for supplied, canonical in aliases.items():
            value = source.get(supplied)
            if isinstance(value, dict):
                fields: dict[str, tuple[float, ...]] = {}
                if "offset" in value:
                    fields["offset"] = _finite_tuple(value["offset"], 3, f"{supplied}.offset")
                matrix_value = value.get("matrix", value.get("scale"))
                if matrix_value is not None:
                    if (
                        isinstance(matrix_value, list)
                        and len(matrix_value) == 3
                        and all(isinstance(row, list) for row in matrix_value)
                    ):
                        matrix_value = [item for row in matrix_value for item in row]
                    fields["matrix"] = _finite_tuple(matrix_value, 9, f"{supplied}.matrix")
                if fields:
                    result[canonical] = fields

        # Accept the existing set_callibration.py YAML spelling.
        for canonical in SENSOR_CALIBRATIONS:
            fields = result.setdefault(canonical, {})
            if f"{canonical}_offset" in source:
                fields["offset"] = _finite_tuple(
                    source[f"{canonical}_offset"], 3, f"{canonical}_offset"
                )
            scale_key = f"{canonical}_scale"
            if scale_key in source:
                value = source[scale_key]
                if isinstance(value, list) and len(value) == 3:
                    value = [item for row in value for item in row]
                fields["matrix"] = _finite_tuple(value, 9, scale_key)
            if not fields:
                result.pop(canonical, None)
        return cls(result)

    def apply(self, base: EffectiveCalibration) -> EffectiveCalibration:
        merged: dict[str, CalibrationValues] = {}
        for name in SENSOR_CALIBRATIONS:
            current = base.values[name]
            override = self.values.get(name, {})
            merged[name] = CalibrationValues(
                cast("tuple[float, float, float]", override.get("offset", current.offset)),
                cast(
                    "tuple[float, float, float, float, float, float, float, float, float]",
                    override.get("matrix", current.matrix),
                ),
            )
        return EffectiveCalibration(merged)


@dataclass(frozen=True)
class EffectiveCalibration:
    """Complete calibration applied to generated artifacts."""

    values: dict[str, CalibrationValues]

    @classmethod
    def identity(cls) -> EffectiveCalibration:
        return cls({name: CalibrationValues.identity() for name in SENSOR_CALIBRATIONS})

    def to_mapping(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "calibration": {name: self.values[name].to_mapping() for name in SENSOR_CALIBRATIONS},
        }


@dataclass(frozen=True)
class LogHeader:
    """Normalized log header independent of its on-disk version."""

    version: str
    device_uid: int
    device_name: str
    communications: tuple[bool, bool, bool, bool]
    firmware_version: str
    frequency_hz: int
    calibration: EffectiveCalibration
    header_size: int


@dataclass(frozen=True)
class LogPacket:
    """One normalized raw sensor packet."""

    packet_index: int
    sensor: str
    identifier: int
    timestamp_ticks: int
    unwrapped_timestamp_ticks: int
    timestamp_s: float
    payload: bytes


@dataclass
class ValidationReport:
    """Result of streaming header and packet validation."""

    path: Path
    version: str | None = None
    packet_count: int = 0
    last_complete_offset: int = 0
    duration_seconds: float = 0.0
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    @property
    def valid(self) -> bool:
        return not self.errors

    def to_mapping(self) -> dict[str, Any]:
        result = asdict(self)
        result["path"] = str(self.path)
        result["valid"] = self.valid
        return result


@dataclass(frozen=True)
class TrimWindow:
    """Accepted inclusive interval in recording-relative seconds."""

    start_s: float
    end_s: float
    source: str = "explicit"
    proposal_id: str | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.start_s) or not math.isfinite(self.end_s):
            raise ValueError("trim times must be finite")
        if self.start_s < 0 or self.end_s < self.start_s:
            raise ValueError("trim window must satisfy 0 <= start <= end")

    def to_mapping(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class TrimProposal:
    """Reviewable phase-detection result."""

    proposal_id: str
    liftoff_s: float
    landed_s: float
    start_s: float
    end_s: float
    confidence: float
    evidence: dict[str, Any]

    def to_mapping(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class BuildRequest:
    """Requested recording build."""

    target_version: str = "1.4"
    force: bool = False


@dataclass(frozen=True)
class BuildResult:
    """Paths and fingerprint produced by a successful build."""

    recording_id: str
    fingerprint: str
    artifacts: dict[str, Path]
    rebuilt: bool
