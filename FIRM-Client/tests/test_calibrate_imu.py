from __future__ import annotations

from dataclasses import dataclass
from itertools import product

import numpy as np
import pytest
from scripts.calibrate_imu import (
    IDENTITY_MATRIX,
    SIDE_COUNT,
    ZERO_OFFSETS,
    CalibrationAborted,
    CalibrationError,
    IMUCalibrationResult,
    PoseCapture,
    apply_calibration,
    ensure_default_calibration,
    fit_accelerometer,
    is_default_imu_calibration,
    solve_imu_calibration,
    summarize_pose,
    wait_for_default_calibration,
)


def canonical_orientations() -> np.ndarray:
    orientations: list[tuple[float, float, float]] = []
    for axis in range(3):
        for sign in (-1.0, 1.0):
            vector = [0.0, 0.0, 0.0]
            vector[axis] = sign
            orientations.append(tuple(vector))

    edge_value = 1.0 / np.sqrt(2.0)
    for zero_axis in range(3):
        nonzero_axes = [axis for axis in range(3) if axis != zero_axis]
        for signs in product((-1.0, 1.0), repeat=2):
            vector = [0.0, 0.0, 0.0]
            for axis, sign in zip(nonzero_axes, signs, strict=True):
                vector[axis] = sign * edge_value
            orientations.append(tuple(vector))

    corner_value = 1.0 / np.sqrt(3.0)
    orientations.extend(
        tuple(sign * corner_value for sign in signs) for signs in product((-1.0, 1.0), repeat=3)
    )
    assert len(orientations) == SIDE_COUNT
    return np.asarray(orientations)


def pose_capture(acceleration: np.ndarray, gyroscope: np.ndarray, count: int = 100) -> PoseCapture:
    return PoseCapture(
        acceleration_mean=acceleration,
        gyroscope_mean=gyroscope,
        acceleration_std=np.zeros(3),
        gyroscope_std=np.zeros(3),
        sample_count=count,
    )


@dataclass
class FakeCalibration:
    imu_accelerometer_offsets: tuple[float, ...] = ZERO_OFFSETS
    imu_accelerometer_scale_matrix: tuple[float, ...] = IDENTITY_MATRIX
    imu_gyroscope_offsets: tuple[float, ...] = ZERO_OFFSETS
    imu_gyroscope_scale_matrix: tuple[float, ...] = IDENTITY_MATRIX


class FakeClient:
    def __init__(
        self,
        calibration: FakeCalibration,
        *,
        accepts_write: bool = True,
        updates_readback: bool = True,
    ) -> None:
        self.calibration = calibration
        self.accepts_write = accepts_write
        self.updates_readback = updates_readback
        self.set_calls: list[tuple[tuple[float, ...], ...]] = []

    def get_calibration(self, timeout_seconds: float) -> FakeCalibration | None:
        """Return the currently stored fake calibration."""
        assert timeout_seconds > 0.0
        return self.calibration

    def set_imu_calibration(
        self,
        accel_offsets: tuple[float, ...],
        accel_matrix: tuple[float, ...],
        gyro_offsets: tuple[float, ...],
        gyro_matrix: tuple[float, ...],
        timeout_seconds: float,
    ) -> bool:
        """Record a calibration write and optionally accept it."""
        assert timeout_seconds > 0.0
        self.set_calls.append((accel_offsets, accel_matrix, gyro_offsets, gyro_matrix))
        if self.accepts_write and self.updates_readback:
            self.calibration = FakeCalibration(
                accel_offsets,
                accel_matrix,
                gyro_offsets,
                gyro_matrix,
            )
        return self.accepts_write


class CalibrationSequenceClient:
    def __init__(self, calibrations: list[FakeCalibration | None]) -> None:
        self.calibrations = calibrations
        self.read_count = 0

    def get_calibration(self, timeout_seconds: float) -> FakeCalibration | None:
        """Return successive startup calibration states."""
        assert timeout_seconds > 0.0
        index = min(self.read_count, len(self.calibrations) - 1)
        self.read_count += 1
        return self.calibrations[index]


def test_order_independent_ellipsoid_fit_recovers_known_calibration() -> None:
    rng = np.random.default_rng(42)
    expected_offset = np.array((0.035, -0.022, 0.014))
    expected_matrix = np.array(
        ((1.018, 0.006, -0.004), (0.006, 0.987, 0.003), (-0.004, 0.003, 1.011))
    )
    orientations = canonical_orientations()
    raw = orientations @ np.linalg.inv(expected_matrix) + expected_offset
    raw += rng.normal(0.0, 1e-4, raw.shape)
    rng.shuffle(raw)

    fit = fit_accelerometer(raw)

    assert fit.offset == pytest.approx(expected_offset, abs=5e-4)
    assert fit.matrix == pytest.approx(expected_matrix, abs=5e-4)
    assert fit.radial_rms_error < 5e-4
    assert fit.radial_max_error < 1e-3


def test_solve_imu_calibration_uses_equal_pose_weighting_for_gyro() -> None:
    expected_offset = np.array((0.01, -0.02, 0.03))
    orientations = canonical_orientations()
    gyro_centroids = np.column_stack(
        (
            np.linspace(0.1, 0.2, SIDE_COUNT),
            np.linspace(-0.3, -0.1, SIDE_COUNT),
            np.linspace(0.02, 0.08, SIDE_COUNT),
        )
    )
    captures = [
        pose_capture(orientation + expected_offset, gyro, count=index + 1)
        for index, (orientation, gyro) in enumerate(zip(orientations, gyro_centroids, strict=True))
    ]

    result = solve_imu_calibration(captures)

    assert result.accelerometer_offset == pytest.approx(expected_offset)
    assert result.accelerometer_matrix == pytest.approx(np.eye(3))
    assert result.gyroscope_offset == pytest.approx(np.mean(gyro_centroids, axis=0))
    assert result.gyroscope_matrix == pytest.approx(np.eye(3))
    assert result.sample_count == sum(range(1, SIDE_COUNT + 1))


def test_summarize_pose_discards_non_finite_rows_and_rejects_empty_data() -> None:
    capture = summarize_pose(
        (
            (1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
            (np.nan, 2.0, 3.0, 4.0, 5.0, 6.0),
        )
    )
    assert capture.sample_count == 1
    assert capture.acceleration_mean == pytest.approx((1.0, 2.0, 3.0))

    with pytest.raises(CalibrationError, match="No IMU samples"):
        summarize_pose(())
    with pytest.raises(CalibrationError, match="No finite"):
        summarize_pose(((np.nan, 0.0, 0.0, 0.0, 0.0, 0.0),))


def test_summarize_pose_rejects_frozen_telemetry() -> None:
    frozen_sample = (0.0, 0.0, 1.0, 0.1, -0.2, 0.3)

    with pytest.raises(CalibrationError, match="telemetry was frozen"):
        summarize_pose((frozen_sample, frozen_sample, frozen_sample))


def test_fit_rejects_degenerate_or_non_finite_orientations() -> None:
    with pytest.raises(CalibrationError, match="rank-deficient"):
        fit_accelerometer(np.tile((1.0, 0.0, 0.0), (SIDE_COUNT, 1)))

    bad = canonical_orientations()
    bad[0, 0] = np.inf
    with pytest.raises(CalibrationError, match="finite"):
        fit_accelerometer(bad)


def test_default_detection_allows_small_float_error() -> None:
    calibration = FakeCalibration(
        imu_accelerometer_offsets=(1e-7, 0.0, 0.0),
        imu_gyroscope_scale_matrix=(1.0, 0.0, 0.0, 0.0, 1.0 + 1e-7, 0.0, 0.0, 0.0, 1.0),
    )
    assert is_default_imu_calibration(calibration)

    calibration.imu_gyroscope_offsets = (0.01, 0.0, 0.0)
    assert not is_default_imu_calibration(calibration)


def test_reset_refusal_does_not_write() -> None:
    client = FakeClient(FakeCalibration(imu_accelerometer_offsets=(0.1, 0.0, 0.0)))

    with pytest.raises(CalibrationAborted):
        ensure_default_calibration(client, 1.0, input_fn=lambda _: "n")

    assert client.set_calls == []


def test_reset_writes_and_verifies_defaults() -> None:
    client = FakeClient(FakeCalibration(imu_accelerometer_offsets=(0.1, 0.0, 0.0)))

    assert ensure_default_calibration(client, 1.0, input_fn=lambda _: "y") is True
    assert len(client.set_calls) == 1
    assert is_default_imu_calibration(client.calibration)


def test_reconnect_waits_for_settings_initialization() -> None:
    not_initialized = FakeCalibration(
        imu_accelerometer_scale_matrix=(0.0,) * 9,
        imu_gyroscope_scale_matrix=(0.0,) * 9,
    )
    client = CalibrationSequenceClient([None, not_initialized, FakeCalibration()])

    calibration = wait_for_default_calibration(client, 1.0, retry_interval=0.0)

    assert is_default_imu_calibration(calibration)
    assert client.read_count == 3


def test_apply_calibration_writes_and_verifies_readback() -> None:
    result = IMUCalibrationResult(
        accelerometer_offset=np.array((0.1, 0.2, 0.3)),
        accelerometer_matrix=np.eye(3) * 1.01,
        gyroscope_offset=np.array((-0.1, 0.05, 0.02)),
        gyroscope_matrix=np.eye(3),
        radial_rms_error=0.001,
        radial_max_error=0.002,
        sample_count=2600,
    )
    client = FakeClient(FakeCalibration())

    apply_calibration(client, result, 1.0)

    assert len(client.set_calls) == 1


def test_apply_calibration_rejects_failed_write() -> None:
    result = IMUCalibrationResult(
        accelerometer_offset=np.zeros(3),
        accelerometer_matrix=np.eye(3),
        gyroscope_offset=np.zeros(3),
        gyroscope_matrix=np.eye(3),
        radial_rms_error=0.0,
        radial_max_error=0.0,
        sample_count=2600,
    )
    client = FakeClient(FakeCalibration(), accepts_write=False)

    with pytest.raises(CalibrationError, match="rejected"):
        apply_calibration(client, result, 1.0)


def test_apply_calibration_rejects_mismatched_readback() -> None:
    result = IMUCalibrationResult(
        accelerometer_offset=np.array((0.1, 0.0, 0.0)),
        accelerometer_matrix=np.eye(3),
        gyroscope_offset=np.zeros(3),
        gyroscope_matrix=np.eye(3),
        radial_rms_error=0.0,
        radial_max_error=0.0,
        sample_count=2600,
    )
    client = FakeClient(FakeCalibration(), updates_readback=False)

    with pytest.raises(CalibrationError, match="readback"):
        apply_calibration(client, result, 1.0)
