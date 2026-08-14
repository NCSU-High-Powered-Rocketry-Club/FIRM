"""Build and execute the native production-source ESKF replay."""

from __future__ import annotations

import hashlib
import json
import shutil
import struct
import subprocess
import time
from dataclasses import dataclass
from datetime import UTC, datetime
from typing import TYPE_CHECKING, Any

import numpy as np
import polars as pl

from .apogee import (
    HPRM_APOGEE_COLUMN,
    HPRM_COAST_TRANSITION_SECONDS,
    add_hprm_apogee_predictions,
)
from .dataset import PreparedDataset, safe_name
from .paths import LAB_ROOT, REPO_ROOT

if TYPE_CHECKING:
    from pathlib import Path

OUTPUT_MAGIC = b"FIRMOUT1"
OUTPUT_VERSION = 1
OUTPUT_COLUMNS = (
    "dt_seconds",
    "eskf_position_z_m",
    "eskf_velocity_z_mps",
    "eskf_quaternion_w",
    "eskf_quaternion_x",
    "eskf_quaternion_y",
    "eskf_quaternion_z",
    "eskf_cov_position_z",
    "eskf_cov_velocity_z",
    "eskf_cov_theta_x",
    "eskf_cov_theta_y",
    "eskf_cov_theta_z",
    "raw_baro_altitude_m",
    "eskf_pressure_coupling",
    "eskf_quaternion_norm",
)
OUTPUT_DTYPE = np.dtype(
    [("timestamp", "<f8"), ("values", "<f4", (len(OUTPUT_COLUMNS),))], align=False
)
FILTER_SOURCE_PATHS = (
    "STM32/Core/Src/data_processing/error_state_kalman_filter.c",
    "STM32/Core/Src/data_processing/eskf_functions.c",
    "STM32/Core/Src/data_processing/eskf_config.c",
    "STM32/Core/Src/data_processing/matrix_helper.c",
    "STM32/Core/Inc/data_processing/error_state_kalman_filter.h",
    "STM32/Core/Inc/data_processing/eskf_functions.h",
    "STM32/Core/Inc/data_processing/eskf_config.h",
    "STM32/Core/Inc/data_processing/matrix_helper.h",
)


@dataclass(frozen=True)
class ReplayResult:
    """Generated result paths and summary metadata."""

    directory: Path
    parquet_path: Path
    metadata_path: Path
    metrics_path: Path
    metadata: dict[str, Any]


def _run_checked(command: list[str], *, cwd: Path = REPO_ROOT) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(command, cwd=cwd, check=True, text=True, capture_output=True)
    except FileNotFoundError as error:
        raise RuntimeError(f"required command is not installed: {command[0]}") from error
    except subprocess.CalledProcessError as error:
        detail = "\n".join(part for part in (error.stdout, error.stderr) if part).strip()
        raise RuntimeError(f"command failed ({' '.join(command)}):\n{detail}") from error


def build_native(build_dir: Path, *, run_tests: bool = True) -> Path:
    """Configure and incrementally build the native replay executable."""
    build_dir = build_dir.resolve()
    build_dir.mkdir(parents=True, exist_ok=True)
    native_source = LAB_ROOT / "native"
    cache_file = build_dir / "CMakeCache.txt"
    if not cache_file.exists():
        command = [
            "cmake",
            "-S",
            str(native_source),
            "-B",
            str(build_dir),
            "-DCMAKE_BUILD_TYPE=Release",
        ]
        if shutil.which("ninja"):
            command.extend(["-G", "Ninja"])
        _run_checked(command)
    _run_checked(["cmake", "--build", str(build_dir), "--config", "Release", "--parallel"])
    if run_tests:
        _run_checked(
            ["ctest", "--test-dir", str(build_dir), "-C", "Release", "--output-on-failure"]
        )

    names = ("firm_eskf_replay.exe", "firm_eskf_replay")
    candidates = [build_dir / name for name in names]
    candidates.extend(build_dir / "Release" / name for name in names)
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise RuntimeError(
        f"native build completed but no replay executable was found under {build_dir}"
    )


def filter_source_hash() -> str:
    """Hash the firmware ESKF implementation and configuration used by a run."""
    digest = hashlib.sha256()
    for relative in FILTER_SOURCE_PATHS:
        path = REPO_ROOT / relative
        digest.update(relative.encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def _git_metadata() -> dict[str, Any]:
    try:
        revision = _run_checked(["git", "rev-parse", "HEAD"]).stdout.strip()
        dirty = bool(_run_checked(["git", "status", "--porcelain"]).stdout.strip())
        branch = _run_checked(["git", "branch", "--show-current"]).stdout.strip()
        return {"revision": revision, "branch": branch, "dirty": dirty}
    except RuntimeError:
        return {"revision": None, "branch": None, "dirty": None}


def _read_native_output(path: Path) -> pl.DataFrame:
    with path.open("rb") as handle:
        header = handle.read(struct.calcsize("<8sIIQ"))
    if len(header) != struct.calcsize("<8sIIQ"):
        raise RuntimeError("native replay produced a truncated output header")
    magic, version, record_size, count = struct.unpack("<8sIIQ", header)
    if magic != OUTPUT_MAGIC or version != OUTPUT_VERSION or record_size != OUTPUT_DTYPE.itemsize:
        raise RuntimeError("native replay produced an incompatible output format")
    records = np.fromfile(path, dtype=OUTPUT_DTYPE, count=count, offset=len(header))
    if len(records) != count:
        raise RuntimeError(f"native replay declared {count} rows but wrote {len(records)}")
    data: dict[str, np.ndarray[Any, Any]] = {"timestamp": records["timestamp"]}
    for index, name in enumerate(OUTPUT_COLUMNS):
        data[name] = records["values"][:, index]
    return pl.DataFrame(data)


def _finite(value: float) -> float | None:
    return value if np.isfinite(value) else None


def calculate_metrics(frame: pl.DataFrame, replay_seconds: float) -> dict[str, Any]:
    """Calculate descriptive filter behavior and performance metrics."""
    timestamp = frame["timestamp"].to_numpy()
    filtered = frame["eskf_position_z_m"].to_numpy()
    velocity = frame["eskf_velocity_z_mps"].to_numpy()
    raw_altitude = frame["raw_baro_altitude_m"].to_numpy()
    quaternion_norm = frame["eskf_quaternion_norm"].to_numpy()
    finite_mask = np.isfinite(filtered) & np.isfinite(velocity) & np.isfinite(quaternion_norm)

    metrics: dict[str, Any] = {
        "rows": frame.height,
        "replay_seconds": replay_seconds,
        "rows_per_second": frame.height / replay_seconds if replay_seconds > 0.0 else None,
        "nonfinite_state_rows": int((~finite_mask).sum()),
    }
    if not finite_mask.any():
        return metrics

    finite_indices = np.flatnonzero(finite_mask)
    apogee_index = finite_indices[int(np.argmax(filtered[finite_mask]))]
    metrics.update(
        {
            "estimated_apogee_m": _finite(float(filtered[apogee_index])),
            "estimated_apogee_time_seconds": _finite(float(timestamp[apogee_index])),
            "max_abs_velocity_mps": _finite(float(np.max(np.abs(velocity[finite_mask])))),
            "max_quaternion_norm_error": _finite(
                float(np.max(np.abs(quaternion_norm[finite_mask] - 1.0)))
            ),
        }
    )

    comparison_mask = finite_mask & np.isfinite(raw_altitude)
    if comparison_mask.any():
        difference = filtered[comparison_mask] - raw_altitude[comparison_mask]
        metrics["filtered_vs_baro_altitude_difference"] = {
            "mean_m": _finite(float(np.mean(difference))),
            "rmse_m": _finite(float(np.sqrt(np.mean(np.square(difference))))),
            "max_abs_m": _finite(float(np.max(np.abs(difference)))),
            "note": (
                "Difference from pressure-derived altitude; barometer altitude is not ground truth."
            ),
        }
        raw_diff = np.diff(raw_altitude[comparison_mask])
        filtered_diff = np.diff(filtered[comparison_mask])
        if raw_diff.size:
            raw_roughness = float(np.sqrt(np.mean(np.square(raw_diff))))
            filtered_roughness = float(np.sqrt(np.mean(np.square(filtered_diff))))
            metrics["sample_to_sample_roughness"] = {
                "raw_baro_altitude_rms_step_m": _finite(raw_roughness),
                "filtered_altitude_rms_step_m": _finite(filtered_roughness),
                "filtered_to_raw_ratio": _finite(filtered_roughness / raw_roughness)
                if raw_roughness > 0.0
                else None,
            }
    if HPRM_APOGEE_COLUMN in frame.columns:
        predictions = frame[HPRM_APOGEE_COLUMN].to_numpy()
        prediction_mask = np.isfinite(predictions)
        hprm_metrics: dict[str, Any] = {
            "rows": int(prediction_mask.sum()),
            "first_m": _finite(float(predictions[prediction_mask][0]))
            if prediction_mask.any()
            else None,
            "last_m": _finite(float(predictions[prediction_mask][-1]))
            if prediction_mask.any()
            else None,
        }
        if prediction_mask.any():
            first_prediction_time = float(timestamp[np.flatnonzero(prediction_mask)[0]])
            steady_mask = prediction_mask & (
                timestamp >= first_prediction_time + HPRM_COAST_TRANSITION_SECONDS
            )
            if steady_mask.any():
                steady_predictions = predictions[steady_mask]
                final_prediction = float(predictions[prediction_mask][-1])
                final_errors = np.abs(steady_predictions - final_prediction)
                hprm_metrics["post_transition"] = {
                    "allowance_seconds": HPRM_COAST_TRANSITION_SECONDS,
                    "rows": int(steady_mask.sum()),
                    "mean_m": _finite(float(np.mean(steady_predictions))),
                    "min_m": _finite(float(np.min(steady_predictions))),
                    "max_m": _finite(float(np.max(steady_predictions))),
                    "range_m": _finite(float(np.ptp(steady_predictions))),
                    "final_prediction_reference_m": _finite(final_prediction),
                    "max_abs_error_from_final_m": _finite(float(np.max(final_errors))),
                    "within_15m_percent": _finite(float(np.mean(final_errors <= 15.0) * 100.0)),
                }

                prediction_indices = np.flatnonzero(prediction_mask)
                last_prediction_index = int(prediction_indices[-1])
                coast_velocity_steps = np.diff(
                    velocity[int(prediction_indices[0]) : last_prediction_index + 1]
                )
                if coast_velocity_steps.size:
                    hprm_metrics["coast_velocity_behavior"] = {
                        "max_abs_step_mps": _finite(
                            float(np.max(np.abs(coast_velocity_steps)))
                        ),
                        "max_increase_step_mps": _finite(float(np.max(coast_velocity_steps))),
                        "increases_over_0_5_mps": int((coast_velocity_steps > 0.5).sum()),
                    }

                if "eskf_pressure_coupling" in frame.columns:
                    pressure_coupling = frame["eskf_pressure_coupling"].to_numpy()
                    # Stay well above the 20 m/s sigmoid transition so the
                    # maximum represents the coast floor, not near-apogee
                    # pressure recoupling.
                    high_speed_coast = prediction_mask & (velocity > 40.0)
                    if high_speed_coast.any():
                        nominal_coast_coupling = float(np.max(pressure_coupling[high_speed_coast]))
                        reliable_pressure = high_speed_coast & (
                            pressure_coupling >= 0.95 * nominal_coast_coupling
                        )
                        unreliable_indices = np.flatnonzero(
                            high_speed_coast & ~reliable_pressure
                        )
                        reliable_indices = np.flatnonzero(reliable_pressure)
                        sustained_rejection = (
                            unreliable_indices.size > 0
                            and timestamp[unreliable_indices[-1]] - first_prediction_time >= 0.5
                        )
                        if (
                            nominal_coast_coupling > 0.0
                            and reliable_indices.size
                            and sustained_rejection
                        ):
                            start_index = int(reliable_indices[0])
                            if unreliable_indices.size:
                                after_last_unreliable = reliable_indices[
                                    reliable_indices > unreliable_indices[-1]
                                ]
                                if after_last_unreliable.size:
                                    start_index = int(after_last_unreliable[0])
                            reacquired = predictions[start_index : last_prediction_index + 1]
                            reacquired = reacquired[np.isfinite(reacquired)]
                            if reacquired.size:
                                reacquired_errors = np.abs(reacquired - final_prediction)
                                hprm_metrics["after_pressure_reacquisition"] = {
                                    "note": (
                                        "Filter pressure reliability recovery; this does not "
                                        "necessarily indicate physical airbrake retraction."
                                    ),
                                    "timestamp_seconds": _finite(float(timestamp[start_index])),
                                    "seconds_after_first_prediction": _finite(
                                        float(timestamp[start_index] - first_prediction_time)
                                    ),
                                    "rows": int(reacquired.size),
                                    "mean_m": _finite(float(np.mean(reacquired))),
                                    "min_m": _finite(float(np.min(reacquired))),
                                    "max_m": _finite(float(np.max(reacquired))),
                                    "max_abs_error_from_final_m": _finite(
                                        float(np.max(reacquired_errors))
                                    ),
                                    "within_15m_percent": _finite(
                                        float(np.mean(reacquired_errors <= 15.0) * 100.0)
                                    ),
                                }
        metrics["hprm_apogee_prediction"] = hprm_metrics
    return metrics


def _result_directory(results_root: Path, prepared: PreparedDataset, source_hash: str) -> Path:
    timestamp = datetime.now(tz=UTC).strftime("%Y%m%dT%H%M%S.%fZ")
    return (
        results_root.resolve()
        / safe_name(prepared.dataset_path.name)
        / f"{timestamp}-{source_hash[:8]}"
    )


def run_replay(
    prepared: PreparedDataset,
    executable: Path,
    results_root: Path,
) -> ReplayResult:
    """Execute the filter and produce a self-describing Parquet result."""
    source_hash = filter_source_hash()
    directory = _result_directory(results_root, prepared, source_hash)
    directory.mkdir(parents=True, exist_ok=False)
    native_output = directory / "native-output.bin"
    started = time.perf_counter()
    process = _run_checked([str(executable), str(prepared.replay_path), str(native_output)])
    replay_seconds = time.perf_counter() - started

    filter_frame = _read_native_output(native_output)
    aligned = pl.scan_parquet(prepared.parquet_path)
    result = (
        aligned.join(filter_frame.lazy(), on="timestamp", how="inner", validate="1:1")
        .sort("timestamp")
        .collect(engine="streaming")
    )
    if result.is_empty():
        raise RuntimeError("native output did not match any prepared timestamps")
    result, apogee_prediction = add_hprm_apogee_predictions(result, prepared.dataset_path)

    parquet_path = directory / "result.parquet"
    result.write_parquet(parquet_path, compression="zstd", statistics=True)
    metrics = calculate_metrics(result, replay_seconds)
    metrics_path = directory / "metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    metadata: dict[str, Any] = {
        "format_version": 1,
        "created_utc": datetime.now(tz=UTC).isoformat(),
        "dataset": prepared.dataset_path.name,
        "prepared_fingerprint": prepared.metadata["fingerprint"],
        "prepared_metadata": str(prepared.metadata_path),
        "firmware_version": prepared.metadata["firmware_version"],
        "filter_source_sha256": source_hash,
        "filter_sources": list(FILTER_SOURCE_PATHS),
        "git": _git_metadata(),
        "native_executable": str(executable),
        "native_summary": process.stderr.strip(),
        "rows": result.height,
        "columns": result.columns,
        "replay_seconds": replay_seconds,
        "hprm_apogee_prediction": (
            apogee_prediction.to_dict() if apogee_prediction is not None else None
        ),
    }
    metadata_path = directory / "run.json"
    metadata_path.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    native_output.unlink(missing_ok=True)

    latest_path = directory.parent / "latest.json"
    latest_temporary = latest_path.with_suffix(".json.tmp")
    latest_temporary.write_text(
        json.dumps({"result_directory": str(directory)}, indent=2) + "\n", encoding="utf-8"
    )
    latest_temporary.replace(latest_path)
    return ReplayResult(directory, parquet_path, metadata_path, metrics_path, metadata)
