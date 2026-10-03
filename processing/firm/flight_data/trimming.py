"""Explicit and reviewable phase-based trim-window selection."""

from __future__ import annotations

import hashlib
import math
from typing import Any

import numpy as np

from .archive import Recording
from .calibration import effective_calibration
from .formats import LogReader, decode_packet
from .models import TrimProposal, TrimWindow


def _first_sustained(times: np.ndarray, mask: np.ndarray, seconds: float) -> int | None:
    start: int | None = None
    for index, selected in enumerate(mask):
        if selected and start is None:
            start = index
        elif not selected:
            start = None
        if start is not None and times[index] - times[start] >= seconds:
            return start
    return None


def propose_phase_trim(
    recording: Recording, *, before_s: float = 5.0, after_s: float = 10.0
) -> TrimProposal:
    """Infer liftoff and landed times, returning a proposal that still requires acceptance."""
    if before_s < 0 or after_s < 0:
        raise ValueError("phase padding must be non-negative")
    calibration = effective_calibration(recording)
    imu_time: list[float] = []
    accel_norm: list[float] = []
    gyro_norm: list[float] = []
    baro_time: list[float] = []
    pressure: list[float] = []
    duration = 0.0
    for packet in LogReader(recording.flight_original).iter_packets():
        duration = packet.timestamp_s
        if packet.sensor == "imu":
            row = decode_packet(packet, calibration)
            accel = tuple(float(row[f"imu_accel_{axis}_g"]) for axis in "xyz")
            gyro = tuple(float(row[f"imu_gyro_{axis}_dps"]) for axis in "xyz")
            imu_time.append(packet.timestamp_s)
            accel_norm.append(math.sqrt(sum(value * value for value in accel)))
            gyro_norm.append(math.sqrt(sum(value * value for value in gyro)))
        elif packet.sensor == "barometer":
            row = decode_packet(packet, calibration)
            baro_time.append(packet.timestamp_s)
            pressure.append(float(row["pressure_pa"]))
    if len(imu_time) < 20 or len(baro_time) < 10:
        raise ValueError("phase detection requires complete IMU and barometer streams")

    imu_t = np.asarray(imu_time)
    accel = np.asarray(accel_norm)
    gyro = np.asarray(gyro_norm)
    baro_t = np.asarray(baro_time)
    pressure_values = np.asarray(pressure)
    positive_pressure = pressure_values[np.isfinite(pressure_values) & (pressure_values > 0)]
    if positive_pressure.size < 10:
        raise ValueError("phase detection found no usable pressure values")
    ground_pressure = float(np.median(positive_pressure[: min(100, positive_pressure.size)]))
    altitude = 44330.0 * (1.0 - np.power(pressure_values / ground_pressure, 0.190294957))

    liftoff_index = _first_sustained(imu_t, accel > 1.5, 0.1)
    if liftoff_index is None:
        raise ValueError("no sustained liftoff acceleration was detected")
    liftoff = float(imu_t[liftoff_index])
    corroborating = altitude[(baro_t >= liftoff) & (baro_t <= liftoff + 2.0)]
    baseline_candidates = altitude[(baro_t >= max(0.0, liftoff - 2.0)) & (baro_t < liftoff)]
    baseline = (
        float(np.median(baseline_candidates)) if baseline_candidates.size else float(altitude[0])
    )
    if corroborating.size == 0 or float(np.nanmax(corroborating)) - baseline < 3.0:
        raise ValueError("liftoff acceleration was not corroborated by a 3 m altitude increase")

    apogee_index = int(np.nanargmax(altitude))
    apogee_time = float(baro_t[apogee_index])
    vertical_speed = np.gradient(altitude, baro_t)
    imu_vertical_speed = np.interp(imu_t, baro_t, vertical_speed)
    landed_mask = (
        (imu_t > apogee_time)
        & (accel >= 0.85)
        & (accel <= 1.15)
        & (gyro <= 3.0)
        & (np.abs(imu_vertical_speed) <= 1.5)
    )
    landed_index = _first_sustained(imu_t, landed_mask, 2.0)
    if landed_index is None:
        raise ValueError("no high-confidence landed interval was detected")
    landed = float(imu_t[landed_index] + 2.0)
    if landed <= liftoff:
        raise ValueError("detected flight phases are incorrectly ordered")
    start = max(0.0, liftoff - before_s)
    end = min(duration, landed + after_s)
    evidence: dict[str, Any] = {
        "liftoff_acceleration_threshold_g": 1.5,
        "liftoff_confirmation_s": 0.1,
        "altitude_corroboration_m": 3.0,
        "apogee_s": apogee_time,
        "landed_confirmation_s": 2.0,
        "landed_acceleration_range_g": [0.85, 1.15],
        "landed_gyro_max_dps": 3.0,
        "landed_pressure_velocity_max_mps": 1.5,
        "before_s": before_s,
        "after_s": after_s,
    }
    identifier = hashlib.sha256(
        f"{recording.dataset_id}\0{liftoff:.6f}\0{landed:.6f}\0{before_s}\0{after_s}".encode()
    ).hexdigest()[:12]
    proposal = TrimProposal(identifier, liftoff, landed, start, end, 0.9, evidence)
    manifest = recording.manifest
    proposals = manifest.setdefault("trim_proposals", {})
    proposals[identifier] = proposal.to_mapping()
    recording.save_manifest(manifest)
    return proposal


def apply_trim_window(
    recording: Recording,
    *,
    start_s: float | None = None,
    end_s: float | None = None,
    proposal_id: str | None = None,
) -> TrimWindow:
    """Accept a phase proposal or save an explicit trim interval."""
    manifest = recording.manifest
    if proposal_id is not None:
        proposal = manifest.get("trim_proposals", {}).get(proposal_id)
        if not isinstance(proposal, dict):
            raise FileNotFoundError(f"trim proposal does not exist: {proposal_id}")
        if float(proposal.get("confidence", 0.0)) < 0.8:
            raise ValueError(
                "phase proposal confidence is too low; apply an explicit --start/--end window"
            )
        window = TrimWindow(
            float(proposal["start_s"]),
            float(proposal["end_s"]),
            source="phase_detection",
            proposal_id=proposal_id,
        )
    else:
        if start_s is None or end_s is None:
            raise ValueError("explicit trimming requires both start and end")
        window = TrimWindow(float(start_s), float(end_s))
    manifest["trim_window"] = window.to_mapping()
    manifest["build"] = {"status": "stale", "reason": "trim window changed"}
    recording.save_manifest(manifest)
    return window
