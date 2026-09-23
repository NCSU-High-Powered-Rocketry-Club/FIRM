"""Interactively calibrate FIRM's ICM45686 using a 26-sided fixture."""

# User-facing failures are intentionally raised with specific actionable messages.
# ruff: noqa: TRY003, TRY300

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from firm_client import FIRMClient

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence

try:
    import numpy as np
except ImportError as exc:  # pragma: no cover - exercised only in an incomplete environment
    raise SystemExit(
        "NumPy is required. From FIRM-Client, run this script with "
        "`uv run --extra extras python scripts/calibrate_imu.py PORT`."
    ) from exc


SIDE_COUNT = 26
DEFAULT_BAUD = 2_000_000
DEFAULT_DURATION_SECONDS = 10.0
DEFAULT_RESPONSE_TIMEOUT_SECONDS = 5.0
DEFAULT_CALIBRATION_TOLERANCE = 1e-6
READBACK_TOLERANCE = 1e-5
MIN_RECOMMENDED_SAMPLES = 10
MIN_ORIENTATION_SEPARATION_G = 0.2

ZERO_OFFSETS = (0.0, 0.0, 0.0)
IDENTITY_MATRIX = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)


class CalibrationError(ValueError):
    """Raised when calibration data cannot produce a safe result."""


class FrozenTelemetryError(CalibrationError):
    """Raised when FIRM transmits packets but its IMU values no longer update."""


class CalibrationAborted(RuntimeError):
    """Raised when the operator cancels the calibration workflow."""


@dataclass(frozen=True)
class PoseCapture:
    """Summary of all valid samples collected on one fixture side."""

    acceleration_mean: np.ndarray
    gyroscope_mean: np.ndarray
    acceleration_std: np.ndarray
    gyroscope_std: np.ndarray
    sample_count: int


@dataclass(frozen=True)
class AccelerometerFit:
    """Order-independent accelerometer ellipsoid fit."""

    offset: np.ndarray
    matrix: np.ndarray
    radial_rms_error: float
    radial_max_error: float


@dataclass(frozen=True)
class IMUCalibrationResult:
    """IMU values in the layout expected by FIRM's calibration command."""

    accelerometer_offset: np.ndarray
    accelerometer_matrix: np.ndarray
    gyroscope_offset: np.ndarray
    gyroscope_matrix: np.ndarray
    radial_rms_error: float
    radial_max_error: float
    sample_count: int


def summarize_pose(samples: Sequence[Sequence[float]]) -> PoseCapture:
    """Discard non-finite rows and summarize six-axis IMU samples."""
    values = np.asarray(samples, dtype=np.float64)
    if values.size == 0:
        raise CalibrationError("No IMU samples were collected")
    if values.ndim != 2 or values.shape[1] != 6:
        raise CalibrationError("IMU samples must have six columns")

    values = values[np.all(np.isfinite(values), axis=1)]
    if len(values) == 0:
        raise CalibrationError("No finite IMU samples were collected")
    if len(values) > 1 and np.all(values == values[0]):
        raise FrozenTelemetryError(
            "IMU telemetry was frozen: every accelerometer and gyroscope sample was identical. "
            "Power-cycle FIRM and restart the calibration"
        )

    acceleration = values[:, :3]
    gyroscope = values[:, 3:]
    return PoseCapture(
        acceleration_mean=np.mean(acceleration, axis=0),
        gyroscope_mean=np.mean(gyroscope, axis=0),
        acceleration_std=np.std(acceleration, axis=0),
        gyroscope_std=np.std(gyroscope, axis=0),
        sample_count=len(values),
    )


def fit_accelerometer(centroids: Sequence[Sequence[float]]) -> AccelerometerFit:
    """Fit ``(sample - offset) * matrix`` to the unit sphere."""
    points = np.asarray(centroids, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 9:
        raise CalibrationError("At least nine three-axis acceleration centroids are required")
    if not np.all(np.isfinite(points)):
        raise CalibrationError("Acceleration centroids must be finite")

    x, y, z = points.T
    design = np.column_stack(
        (x * x, y * y, z * z, 2.0 * x * y, 2.0 * x * z, 2.0 * y * z, 2.0 * x, 2.0 * y, 2.0 * z)
    )
    solution, _, rank, _ = np.linalg.lstsq(design, np.ones(len(points)), rcond=None)
    if rank < 9:
        raise CalibrationError(
            "Acceleration orientations are rank-deficient. The captures do not contain enough "
            "distinct live orientations; check for frozen IMU telemetry or repeated fixture sides"
        )

    q_matrix = np.array(
        (
            (solution[0], solution[3], solution[4]),
            (solution[3], solution[1], solution[5]),
            (solution[4], solution[5], solution[2]),
        ),
        dtype=np.float64,
    )
    linear = solution[6:9]
    if not np.all(np.isfinite(q_matrix)) or not np.all(np.isfinite(linear)):
        raise CalibrationError("Ellipsoid fit produced non-finite coefficients")

    eigenvalues = np.linalg.eigvalsh(q_matrix)
    if np.any(eigenvalues <= 0.0):
        raise CalibrationError(
            "Acceleration fit is not a positive-definite ellipsoid; repeat the fixture captures"
        )

    try:
        offset = -np.linalg.solve(q_matrix, linear)
    except np.linalg.LinAlgError as exc:
        raise CalibrationError("Acceleration ellipsoid is singular") from exc

    radius_squared = 1.0 + float(offset @ q_matrix @ offset)
    if not math.isfinite(radius_squared) or radius_squared <= 0.0:
        raise CalibrationError("Acceleration ellipsoid has an invalid radius")

    normalized_shape = q_matrix / radius_squared
    eigenvalues, eigenvectors = np.linalg.eigh(normalized_shape)
    if np.any(eigenvalues <= 0.0) or not np.all(np.isfinite(eigenvalues)):
        raise CalibrationError("Acceleration correction matrix is not positive-definite")

    matrix = (eigenvectors * np.sqrt(eigenvalues)) @ eigenvectors.T
    matrix = (matrix + matrix.T) * 0.5
    corrected = (points - offset) @ matrix
    radial_errors = np.linalg.norm(corrected, axis=1) - 1.0

    if not np.all(np.isfinite(offset)) or not np.all(np.isfinite(matrix)):
        raise CalibrationError("Acceleration calibration contains non-finite values")

    return AccelerometerFit(
        offset=offset,
        matrix=matrix,
        radial_rms_error=float(np.sqrt(np.mean(radial_errors * radial_errors))),
        radial_max_error=float(np.max(np.abs(radial_errors))),
    )


def solve_imu_calibration(captures: Sequence[PoseCapture]) -> IMUCalibrationResult:
    """Calculate accelerometer calibration and stationary gyroscope bias."""
    if len(captures) != SIDE_COUNT:
        raise CalibrationError(f"Expected {SIDE_COUNT} fixture-side captures, got {len(captures)}")

    acceleration_centroids = np.vstack([capture.acceleration_mean for capture in captures])
    gyroscope_centroids = np.vstack([capture.gyroscope_mean for capture in captures])
    accelerometer = fit_accelerometer(acceleration_centroids)
    gyroscope_offset = np.mean(gyroscope_centroids, axis=0)
    if not np.all(np.isfinite(gyroscope_offset)):
        raise CalibrationError("Gyroscope calibration contains non-finite values")

    return IMUCalibrationResult(
        accelerometer_offset=accelerometer.offset,
        accelerometer_matrix=accelerometer.matrix,
        gyroscope_offset=gyroscope_offset,
        gyroscope_matrix=np.eye(3, dtype=np.float64),
        radial_rms_error=accelerometer.radial_rms_error,
        radial_max_error=accelerometer.radial_max_error,
        sample_count=sum(capture.sample_count for capture in captures),
    )


def is_default_imu_calibration(
    calibration: Any, tolerance: float = DEFAULT_CALIBRATION_TOLERANCE
) -> bool:
    """Return whether the stored accelerometer and gyro values are identity calibration."""
    expected_matrix = np.asarray(IDENTITY_MATRIX)
    return all(
        (
            np.allclose(
                calibration.imu_accelerometer_offsets, ZERO_OFFSETS, rtol=0.0, atol=tolerance
            ),
            np.allclose(
                calibration.imu_accelerometer_scale_matrix,
                expected_matrix,
                rtol=0.0,
                atol=tolerance,
            ),
            np.allclose(calibration.imu_gyroscope_offsets, ZERO_OFFSETS, rtol=0.0, atol=tolerance),
            np.allclose(
                calibration.imu_gyroscope_scale_matrix,
                expected_matrix,
                rtol=0.0,
                atol=tolerance,
            ),
        )
    )


def ask_yes_no(
    prompt: str,
    *,
    default: bool,
    input_fn: Callable[[str], str] = input,
) -> bool:
    """Ask a yes/no question until the operator provides a valid answer."""
    suffix = " [Y/n] " if default else " [y/N] "
    while True:
        answer = input_fn(prompt + suffix).strip().lower()
        if not answer:
            return default
        if answer in {"y", "yes"}:
            return True
        if answer in {"n", "no"}:
            return False
        print("Please answer yes or no.")


def _format_vector(values: Sequence[float]) -> str:
    return "(" + ", ".join(f"{float(value):.9g}" for value in values) + ")"


def _format_matrix(values: Sequence[float] | np.ndarray) -> str:
    matrix = np.asarray(values, dtype=np.float64).reshape(3, 3)
    return "\n".join("  [" + ", ".join(f"{value:.9g}" for value in row) + "]" for row in matrix)


def print_stored_imu_calibration(calibration: Any) -> None:
    print("Stored IMU calibration is not at its defaults:")
    print("  Accelerometer offset:", _format_vector(calibration.imu_accelerometer_offsets))
    print("  Accelerometer matrix:\n" + _format_matrix(calibration.imu_accelerometer_scale_matrix))
    print("  Gyroscope offset:", _format_vector(calibration.imu_gyroscope_offsets))
    print("  Gyroscope matrix:\n" + _format_matrix(calibration.imu_gyroscope_scale_matrix))


def read_calibration(client: Any, response_timeout: float) -> Any:
    calibration = client.get_calibration(timeout_seconds=response_timeout)
    if calibration is None:
        raise CalibrationError("Timed out while reading calibration from FIRM")
    return calibration


def wait_for_default_calibration(
    client: Any,
    response_timeout: float,
    *,
    retry_interval: float = 0.25,
) -> Any:
    """Wait for startup to load default IMU calibration from flash."""
    deadline = time.monotonic() + response_timeout
    last_calibration = None
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break

        calibration = client.get_calibration(timeout_seconds=min(1.0, remaining))
        if calibration is not None:
            last_calibration = calibration
            if is_default_imu_calibration(calibration):
                return calibration

        remaining = deadline - time.monotonic()
        if remaining > 0.0:
            time.sleep(min(retry_interval, remaining))

    if last_calibration is None:
        raise CalibrationError(
            "Timed out while verifying default IMU calibration after reconnecting"
        )
    raise CalibrationError(
        "IMU calibration was not at defaults after waiting for firmware startup to finish"
    )


def ensure_default_calibration(
    client: Any,
    response_timeout: float,
    *,
    input_fn: Callable[[str], str] = input,
) -> bool:
    """Reset non-default IMU calibration and return whether a power-cycle is needed."""
    calibration = read_calibration(client, response_timeout)
    if is_default_imu_calibration(calibration):
        print("Stored accelerometer and gyroscope calibration is already at the defaults.")
        return False

    print_stored_imu_calibration(calibration)
    if not ask_yes_no(
        "Reset accelerometer and gyroscope calibration before collecting data?",
        default=False,
        input_fn=input_fn,
    ):
        raise CalibrationAborted("Calibration cancelled without resetting existing IMU values")

    accepted = client.set_imu_calibration(
        ZERO_OFFSETS,
        IDENTITY_MATRIX,
        ZERO_OFFSETS,
        IDENTITY_MATRIX,
        timeout_seconds=response_timeout,
    )
    if not accepted:
        raise CalibrationError("FIRM rejected or timed out while resetting IMU calibration")
    if not is_default_imu_calibration(read_calibration(client, response_timeout)):
        raise CalibrationError("IMU reset readback did not match the requested defaults")

    print("Default IMU calibration was stored and verified.")
    return True


def _packet_values(packet: Any) -> tuple[float, float, float, float, float, float]:
    return (
        float(packet.raw_acceleration_x_gs),
        float(packet.raw_acceleration_y_gs),
        float(packet.raw_acceleration_z_gs),
        float(packet.raw_angular_rate_x_deg_per_s),
        float(packet.raw_angular_rate_y_deg_per_s),
        float(packet.raw_angular_rate_z_deg_per_s),
    )


def collect_pose(client: Any, duration_seconds: float) -> PoseCapture:
    """Drain movement packets, then collect one timed stationary pose."""
    client.get_data_packets()
    deadline = time.monotonic() + duration_seconds
    samples: list[tuple[float, float, float, float, float, float]] = []
    next_update = 0.0

    while True:
        now = time.monotonic()
        if now >= deadline:
            break
        for packet in client.get_data_packets():
            values = _packet_values(packet)
            if all(math.isfinite(value) for value in values):
                samples.append(values)
        remaining = deadline - now
        if now >= next_update:
            print(f"\rCollecting... {remaining:4.1f} seconds remaining", end="", flush=True)
            next_update = now + 0.25
        time.sleep(min(0.01, max(remaining, 0.0)))

    # Drain packets that arrived just before the deadline but were not observed by the last poll.
    for packet in client.get_data_packets():
        values = _packet_values(packet)
        if all(math.isfinite(value) for value in values):
            samples.append(values)
    print("\rCollection complete.                         ")
    return summarize_pose(samples)


def collect_fixture_captures(
    client: Any,
    duration_seconds: float,
    *,
    input_fn: Callable[[str], str] = input,
) -> list[PoseCapture]:
    """Guide the operator through all numbered fixture sides."""
    captures: list[PoseCapture] = []
    side = 1
    while side <= SIDE_COUNT:
        input_fn(
            f"\nPlace the fixture on side {side} of {SIDE_COUNT}. "
            "Keep it stationary, then press Enter to collect."
        )
        try:
            capture = collect_pose(client, duration_seconds)
        except FrozenTelemetryError:
            raise
        except CalibrationError as exc:
            print(f"Side {side} was not captured: {exc}. Please try this side again.")
            continue

        print(f"  Valid samples: {capture.sample_count}")
        print(f"  Mean accel (g): {_format_vector(capture.acceleration_mean)}")
        print(f"  Mean gyro (deg/s): {_format_vector(capture.gyroscope_mean)}")
        print(f"  Accel std (g): {_format_vector(capture.acceleration_std)}")
        print(f"  Gyro std (deg/s): {_format_vector(capture.gyroscope_std)}")

        if captures:
            nearest_distance = min(
                float(np.linalg.norm(capture.acceleration_mean - previous.acceleration_mean))
                for previous in captures
            )
            if nearest_distance < MIN_ORIENTATION_SEPARATION_G:
                print(
                    f"Side {side} is only {nearest_distance:.6g} g from a previous capture. "
                    "The IMU stream may be frozen or this fixture side may have already been used. "
                    "Please verify that live acceleration changes while moving FIRM, then retry "
                    "this side."
                )
                continue

        if capture.sample_count < MIN_RECOMMENDED_SAMPLES and ask_yes_no(
            f"Only {capture.sample_count} valid samples were received. Retry side {side}?",
            default=True,
            input_fn=input_fn,
        ):
            continue

        captures.append(capture)
        side += 1
    return captures


def print_result(result: IMUCalibrationResult) -> None:
    print("\nCalibration result")
    print("==================")
    print(f"Total samples: {result.sample_count}")
    print(f"Acceleration radial RMS error: {result.radial_rms_error:.9g} g")
    print(f"Acceleration radial max error: {result.radial_max_error:.9g} g")
    print("\nAccelerometer offset (g):", _format_vector(result.accelerometer_offset))
    print("Accelerometer 3x3 matrix:\n" + _format_matrix(result.accelerometer_matrix))
    print(
        "Accelerometer matrix, flat row-major:", _format_vector(result.accelerometer_matrix.ravel())
    )
    print("\nGyroscope offset (deg/s):", _format_vector(result.gyroscope_offset))
    print("Gyroscope 3x3 matrix:\n" + _format_matrix(result.gyroscope_matrix))
    print("Gyroscope matrix, flat row-major:", _format_vector(result.gyroscope_matrix.ravel()))


def _result_as_command_values(
    result: IMUCalibrationResult,
) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    return (
        tuple(float(value) for value in result.accelerometer_offset),
        tuple(float(value) for value in result.accelerometer_matrix.ravel()),
        tuple(float(value) for value in result.gyroscope_offset),
        tuple(float(value) for value in result.gyroscope_matrix.ravel()),
    )


def calibration_matches_result(
    calibration: Any,
    result: IMUCalibrationResult,
    tolerance: float = READBACK_TOLERANCE,
) -> bool:
    accel_offset, accel_matrix, gyro_offset, gyro_matrix = _result_as_command_values(result)
    comparisons = (
        (calibration.imu_accelerometer_offsets, accel_offset),
        (calibration.imu_accelerometer_scale_matrix, accel_matrix),
        (calibration.imu_gyroscope_offsets, gyro_offset),
        (calibration.imu_gyroscope_scale_matrix, gyro_matrix),
    )
    return all(
        np.allclose(actual, expected, rtol=tolerance, atol=tolerance)
        for actual, expected in comparisons
    )


def apply_calibration(client: Any, result: IMUCalibrationResult, response_timeout: float) -> None:
    accel_offset, accel_matrix, gyro_offset, gyro_matrix = _result_as_command_values(result)
    accepted = client.set_imu_calibration(
        accel_offset,
        accel_matrix,
        gyro_offset,
        gyro_matrix,
        timeout_seconds=response_timeout,
    )
    if not accepted:
        raise CalibrationError("FIRM rejected or timed out while applying IMU calibration")
    if not calibration_matches_result(read_calibration(client, response_timeout), result):
        raise CalibrationError("Applied IMU calibration did not match its readback")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Calibrate FIRM's accelerometer and stationary gyroscope bias on a 26-sided fixture"
        )
    )
    parser.add_argument("port", help='Serial port, for example "COM8" or "/dev/ttyACM0"')
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_SECONDS,
        help="Seconds to collect on each fixture side (default: 10)",
    )
    parser.add_argument(
        "--response-timeout",
        type=float,
        default=DEFAULT_RESPONSE_TIMEOUT_SECONDS,
        help="Seconds to wait for command responses (default: 5)",
    )
    return parser


def _connect(port: str, baud: int) -> Any:
    client = FIRMClient(port, baud, timeout=0.1)
    client.start()
    return client


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.baud <= 0:
        parser.error("--baud must be positive")
    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    if args.response_timeout <= 0.0:
        parser.error("--response-timeout must be positive")

    client = None
    try:
        print(f"Connecting to {args.port} at {args.baud} baud...")
        client = _connect(args.port, args.baud)
        reset_performed = ensure_default_calibration(client, args.response_timeout)

        if reset_performed:
            client.stop()
            client = None
            input(
                "\nUnplug FIRM, plug it back in so the default calibration takes effect, "
                "then press Enter to reconnect."
            )
            client = _connect(args.port, args.baud)
            print("Waiting for firmware settings initialization...")
            wait_for_default_calibration(client, args.response_timeout)
            print("Reconnected and verified default IMU calibration.")

        captures = collect_fixture_captures(client, args.duration)
        result = solve_imu_calibration(captures)
        print_result(result)

        if ask_yes_no("Apply this calibration to FIRM?", default=False):
            apply_calibration(client, result, args.response_timeout)
            print("\nCalibration was stored and verified.")
            print("Unplug and replug FIRM before using it so the new calibration takes effect.")
        else:
            print("Calibration was not written to FIRM.")
        return 0
    except CalibrationAborted as exc:
        print(f"\n{exc}")
        return 1
    except (CalibrationError, OSError) as exc:
        print(f"\nCalibration failed: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nCalibration cancelled.")
        return 130
    finally:
        if client is not None:
            client.stop()


if __name__ == "__main__":
    raise SystemExit(main())
