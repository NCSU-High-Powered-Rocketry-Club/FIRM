"""Optional HPRM apogee predictions derived from replayed ESKF state."""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import asdict, dataclass
from importlib.metadata import version
from typing import TYPE_CHECKING, Any

import numpy as np
import polars as pl
from hprm import AdaptiveTimeStep, InitialState1DOF, OdeMethod, Rocket

if TYPE_CHECKING:
    from pathlib import Path

ROCKET_PROPERTIES_FILENAME = "rocket_properties.json"
HPRM_APOGEE_COLUMN = "hprm_predicted_apogee_m"
TAKEOFF_VELOCITY_MPS = 10.0
HPRM_DT_MAX_SECONDS = 1.0
HPRM_COAST_TRANSITION_SECONDS = 0.5
COAST_MINIMUM_VELOCITY_DROP_MPS = 1.0
COAST_CONFIRMATION_SECONDS = 0.1


@dataclass(frozen=True)
class RocketProperties:
    """Physical properties required by HPRM's one-dimensional model."""

    cd: float
    dry_mass_kg: float
    cross_sectional_area_m2: float


@dataclass(frozen=True)
class ApogeePredictionSummary:
    """Provenance and timing for an optional prediction series."""

    properties_file: str
    properties_sha256: str
    rocket_properties: RocketProperties
    hprm_version: str
    model: str
    ode_method: str
    adaptive_dt_max_seconds: float
    prediction_rows: int
    first_prediction_timestamp_seconds: float | None
    last_prediction_timestamp_seconds: float | None
    calculation_seconds: float

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable representation."""
        return asdict(self)


def _finite_number(document: dict[str, Any], key: str, path: Path) -> float:
    value = document.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{path} field {key!r} must be a finite number")  # noqa: TRY004
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{path} field {key!r} must be a finite number")
    return result


def load_rocket_properties(dataset_path: Path) -> tuple[RocketProperties, str] | None:
    """Load a dataset's optional HPRM rocket properties and source hash."""
    path = dataset_path / ROCKET_PROPERTIES_FILENAME
    if not path.is_file():
        return None
    raw = path.read_bytes()
    try:
        document = json.loads(raw)
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid JSON in {path}: {error.msg}") from error
    if not isinstance(document, dict):
        raise ValueError(f"{path} must contain a JSON object")  # noqa: TRY004
    rocket = document.get("rocket", document)
    if not isinstance(rocket, dict):
        raise ValueError(f"{path} field 'rocket' must be a JSON object")  # noqa: TRY004

    properties = RocketProperties(
        cd=_finite_number(rocket, "rocket_Cd", path),
        dry_mass_kg=_finite_number(rocket, "rocket_dry_mass_kg", path),
        cross_sectional_area_m2=_finite_number(rocket, "rocket_cross_sectional_area_m2", path),
    )
    if properties.cd < 0.0:
        raise ValueError(f"{path} field 'rocket_Cd' must be non-negative")
    if properties.dry_mass_kg <= 0.0:
        raise ValueError(f"{path} field 'rocket_dry_mass_kg' must be positive")
    if properties.cross_sectional_area_m2 <= 0.0:
        raise ValueError(f"{path} field 'rocket_cross_sectional_area_m2' must be positive")
    return properties, hashlib.sha256(raw).hexdigest()


def add_hprm_apogee_predictions(
    frame: pl.DataFrame, dataset_path: Path
) -> tuple[pl.DataFrame, ApogeePredictionSummary | None]:
    """Append coast-phase HPRM predictions when a properties file is present."""
    loaded = load_rocket_properties(dataset_path)
    if loaded is None:
        return frame, None
    properties, properties_sha256 = loaded
    rocket = Rocket(
        properties.dry_mass_kg,
        properties.cd,
        properties.cross_sectional_area_m2,
        0.0,
        0.0,
        0.0,
        0.0,
    )

    timestamps = frame["timestamp"].to_numpy()
    altitudes = frame["eskf_position_z_m"].to_numpy()
    velocities = frame["eskf_velocity_z_mps"].to_numpy()
    predictions = np.full(frame.height, np.nan, dtype=np.float64)
    prediction_indices: list[int] = []
    in_motor_burn = False
    in_coast = False
    maximum_velocity = -math.inf
    coast_candidate_timestamp: float | None = None
    started = time.perf_counter()

    for index, (altitude, velocity) in enumerate(zip(altitudes, velocities, strict=True)):
        if not (math.isfinite(altitude) and math.isfinite(velocity)):
            continue
        if not in_motor_burn:
            if velocity > TAKEOFF_VELOCITY_MPS:
                in_motor_burn = True
                maximum_velocity = velocity
            continue
        if not in_coast:
            if velocity >= maximum_velocity:
                maximum_velocity = velocity
                coast_candidate_timestamp = None
                continue
            if maximum_velocity - velocity < COAST_MINIMUM_VELOCITY_DROP_MPS:
                coast_candidate_timestamp = None
                continue
            if coast_candidate_timestamp is None:
                coast_candidate_timestamp = float(timestamps[index])
                continue
            if float(timestamps[index]) - coast_candidate_timestamp < COAST_CONFIRMATION_SECONDS:
                continue
            in_coast = True
        if velocity <= 0.0:
            break

        timestep = AdaptiveTimeStep.default()
        timestep.dt_max = HPRM_DT_MAX_SECONDS
        initial_state = InitialState1DOF(float(altitude), float(velocity))
        predictions[index] = rocket.predict_apogee_1dof(
            initial_state,
            OdeMethod.RK45,
            timestep,
        )
        prediction_indices.append(index)

    calculation_seconds = time.perf_counter() - started
    result = frame.with_columns(pl.Series(HPRM_APOGEE_COLUMN, predictions))
    first_timestamp = float(timestamps[prediction_indices[0]]) if prediction_indices else None
    last_timestamp = float(timestamps[prediction_indices[-1]]) if prediction_indices else None
    summary = ApogeePredictionSummary(
        properties_file=str(dataset_path / ROCKET_PROPERTIES_FILENAME),
        properties_sha256=properties_sha256,
        rocket_properties=properties,
        hprm_version=version("hprm"),
        model="OneDOF",
        ode_method="RK45",
        adaptive_dt_max_seconds=HPRM_DT_MAX_SECONDS,
        prediction_rows=len(prediction_indices),
        first_prediction_timestamp_seconds=first_timestamp,
        last_prediction_timestamp_seconds=last_timestamp,
        calculation_seconds=calculation_seconds,
    )
    return result, summary
