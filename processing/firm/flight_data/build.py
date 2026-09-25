"""Reproducible migrated-log and Parquet build pipeline."""

from __future__ import annotations

import json
import shutil
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .archive import Recording, sha256_file, write_yaml_atomic
from .calibration import effective_calibration
from .decoding import decode_to_parquet, validate_parquet_outputs
from .fingerprint import BUILD_SCHEMA_VERSION, DECODER_SCHEMA_VERSION, build_fingerprint
from .formats import (
    LATEST_VERSION,
    TARGET_HARDWARE,
    LogReader,
    LogWriter,
    calibration_for_hardware,
)
from .models import BuildRequest, BuildResult, TrimWindow


def _trim_window(manifest: dict[str, Any]) -> TrimWindow | None:
    value = manifest.get("trim_window")
    if not isinstance(value, dict):
        return None
    return TrimWindow(
        float(value["start_s"]),
        float(value["end_s"]),
        str(value.get("source", "explicit")),
        value.get("proposal_id"),
    )


def _verify_originals(recording: Recording) -> None:
    manifest = recording.manifest
    expected = manifest.get("sources", {})
    sources = [("flight", recording.flight_original)]
    if "magnetometer_calibration" in expected:
        sources.append(("magnetometer_calibration", recording.mag_cal_original))
    for name, path in sources:
        if not path.is_file():
            raise FileNotFoundError(f"managed original is missing: {path}")
        actual = sha256_file(path)
        if actual != expected[name].get("sha256"):
            raise ValueError(f"immutable original changed: {path}")


def _promote(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".incoming")
    shutil.copyfile(source, temporary)
    temporary.replace(destination)


def _timestamp_anchors(source: Path, trim_window: TrimWindow | None) -> tuple[int, int]:
    """Return the recording origin and first retained unwrapped counter."""
    origin: int | None = None
    first_retained: int | None = None
    for packet in LogReader(source).iter_packets():
        if origin is None:
            origin = packet.unwrapped_timestamp_ticks
        if trim_window is None or (trim_window.start_s <= packet.timestamp_s <= trim_window.end_s):
            first_retained = packet.unwrapped_timestamp_ticks
            break
    if origin is None or first_retained is None:
        raise ValueError("selected interval contains no complete packets")
    return origin, first_retained


def build_recording(recording: Recording, request: BuildRequest | None = None) -> BuildResult:
    """Build and validate every current artifact for a recording."""
    request = request or BuildRequest()
    if request.target_version not in {"latest", LATEST_VERSION}:
        raise ValueError(f"only target log version {LATEST_VERSION} is currently writable")
    target_version = LATEST_VERSION
    _verify_originals(recording)
    manifest = recording.manifest
    source_hardware = recording.source_hardware
    fingerprint = build_fingerprint(recording, target_version)
    if (
        not request.force
        and manifest.get("build", {}).get("fingerprint") == fingerprint
        and recording.is_current
    ):
        artifacts = {
            "flight_log": recording.flight_derived,
            **{
                name: recording.parquet_path(name)
                for name in ("barometer", "imu", "magnetometer", "high-g")
            },
        }
        if recording.mag_cal_original.is_file():
            artifacts["magnetometer_calibration_log"] = recording.mag_cal_derived
            artifacts["magnetometer_calibration"] = recording.parquet_path(
                "magnetometer-calibration"
            )
        return BuildResult(recording.dataset_id, fingerprint, artifacts, False)

    flight_validation = LogReader(recording.flight_original).validate()
    if not flight_validation.valid:
        raise ValueError("flight log validation failed: " + "; ".join(flight_validation.errors))
    mag_validation = None
    if recording.mag_cal_original.is_file():
        mag_validation = LogReader(recording.mag_cal_original).validate()
        if not mag_validation.valid:
            raise ValueError(
                "magnetometer-calibration log validation failed: "
                + "; ".join(mag_validation.errors)
            )

    source_calibration = effective_calibration(recording)
    calibration = calibration_for_hardware(source_calibration, source_hardware)
    trim_window = _trim_window(manifest)
    staging = recording.path / f".build-{uuid.uuid4().hex}"
    staging_derived = staging / "derived"
    staging_decoded = staging / "decoded"
    staging_derived.mkdir(parents=True)
    try:
        flight_origin_ticks, flight_first_ticks = _timestamp_anchors(
            recording.flight_original, trim_window
        )
        flight_output = staging_derived / "flight.frm"
        flight_packets = LogWriter.migrate(
            recording.flight_original,
            flight_output,
            source_calibration,
            trim_window,
            source_hardware=source_hardware,
        )
        migrated_validation = LogReader(flight_output).validate()
        if not migrated_validation.valid:
            raise ValueError(
                "generated flight log failed validation: " + "; ".join(migrated_validation.errors)
            )
        metadata = {
            "schema_version": str(DECODER_SCHEMA_VERSION),
            "recording": recording.dataset_id,
            "build_fingerprint": fingerprint,
            "source_sha256": manifest["sources"]["flight"]["sha256"],
            "target_log_version": target_version,
            "source_hardware": source_hardware,
            "target_hardware": TARGET_HARDWARE,
            "artifact_role": "flight",
            "calibration_override_sha256": (manifest.get("calibration_override") or {}).get(
                "sha256", ""
            ),
        }
        parquet_paths, packet_counts = decode_to_parquet(
            flight_output,
            staging_decoded,
            calibration,
            trimmed_start_s=trim_window.start_s if trim_window else 0.0,
            metadata=metadata,
            time_origin_ticks=flight_origin_ticks,
            first_unwrapped_ticks=flight_first_ticks,
        )
        validate_parquet_outputs(parquet_paths, packet_counts)

        mag_output: Path | None = None
        mag_parquet: Path | None = None
        mag_packets = 0
        mag_counts: dict[str, int] = {}
        if recording.mag_cal_original.is_file():
            mag_origin_ticks, mag_first_ticks = _timestamp_anchors(recording.mag_cal_original, None)
            mag_output = staging_derived / "magnetometer-calibration.frm"
            mag_packets = LogWriter.migrate(
                recording.mag_cal_original,
                mag_output,
                source_calibration,
                source_hardware=source_hardware,
            )
            generated_mag_validation = LogReader(mag_output).validate()
            if not generated_mag_validation.valid:
                raise ValueError("generated magnetometer-calibration log failed validation")
            mag_paths, mag_counts = decode_to_parquet(
                mag_output,
                staging_decoded,
                calibration,
                metadata={
                    **metadata,
                    "source_sha256": manifest["sources"]["magnetometer_calibration"]["sha256"],
                    "artifact_role": "magnetometer_calibration",
                },
                magnetometer_calibration=True,
                time_origin_ticks=mag_origin_ticks,
                first_unwrapped_ticks=mag_first_ticks,
            )
            mag_parquet = mag_paths["magnetometer"]
            validate_parquet_outputs(mag_paths, mag_counts)

        effective_path = staging_derived / "effective-calibration.yaml"
        write_yaml_atomic(effective_path, calibration.to_mapping())
        created = datetime.now(tz=timezone.utc).isoformat()
        build_document: dict[str, Any] = {
            "schema_version": BUILD_SCHEMA_VERSION,
            "created_utc": created,
            "recording": recording.dataset_id,
            "fingerprint": fingerprint,
            "target_log_version": target_version,
            "source_hardware": source_hardware,
            "target_hardware": TARGET_HARDWARE,
            "trim_window": trim_window.to_mapping() if trim_window else None,
            "flight_packets": flight_packets,
            "magnetometer_calibration_packets": mag_packets,
            "parquet_rows": {
                **packet_counts,
                "magnetometer_calibration": mag_counts.get("magnetometer", 0),
            },
        }
        staged_artifacts = [flight_output, effective_path, *parquet_paths.values()]
        if mag_output is not None:
            staged_artifacts.append(mag_output)
        if mag_parquet is not None:
            staged_artifacts.append(mag_parquet)
        build_document["artifacts"] = {
            path.relative_to(staging).as_posix(): {
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
            for path in staged_artifacts
        }
        build_json = staging_derived / "build.json"
        build_json.write_text(
            json.dumps(build_document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

        destinations: dict[str, tuple[Path, Path]] = {
            "flight_log": (flight_output, recording.flight_derived),
            "effective_calibration": (
                effective_path,
                recording.path / "derived" / "effective-calibration.yaml",
            ),
            "build_metadata": (build_json, recording.build_metadata_path),
        }
        for sensor, path in parquet_paths.items():
            output_name = "high-g" if sensor == "high_g" else sensor
            destinations[output_name] = (path, recording.parquet_path(output_name))
        if mag_output is not None and mag_parquet is not None:
            destinations["magnetometer_calibration_log"] = (
                mag_output,
                recording.mag_cal_derived,
            )
            destinations["magnetometer_calibration"] = (
                mag_parquet,
                recording.parquet_path("magnetometer-calibration"),
            )
        for source, destination in destinations.values():
            _promote(source, destination)

        manifest = recording.manifest
        artifact_status = {
            name: {
                "status": "current",
                "path": destination.relative_to(recording.path).as_posix(),
                "sha256": sha256_file(source),
                "size_bytes": source.stat().st_size,
            }
            for name, (source, destination) in destinations.items()
        }
        manifest["build"] = {
            "status": "current",
            "fingerprint": fingerprint,
            "target_log_version": target_version,
            "source_hardware": source_hardware,
            "target_hardware": TARGET_HARDWARE,
            "created_utc": created,
            "metadata": "derived/build.json",
            "artifacts": artifact_status,
        }
        recording.save_manifest(manifest)
        return BuildResult(
            recording.dataset_id,
            fingerprint,
            {name: destination for name, (_source, destination) in destinations.items()},
            True,
        )
    finally:
        shutil.rmtree(staging, ignore_errors=True)
