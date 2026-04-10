"""ctypes adapter for calling the host-side ESKF setup API."""

from __future__ import annotations

from ctypes import CDLL, POINTER, byref, c_float, c_int, c_uint32
from pathlib import Path

try:
    from .build_filter import build_filter_shared_library
except ImportError:
    from build_filter import build_filter_shared_library

STATUS_OK = 0
STATUS_INVALID_ARGUMENT = 1
STATUS_NOT_INITIALIZED = 2
STATUS_INIT_FAILED = 3
STATUS_INSUFFICIENT_ACCUMULATION = 4

ORIENTATION_V1 = 1
ORIENTATION_V2 = 2


class FilterHarnessError(RuntimeError):
    """Raised when the host filter API returns a nonzero status."""


class FilterHarness:
    """High-level Python wrapper around the setup.c host API."""

    def __init__(self, *, auto_build: bool = True, force_rebuild: bool = False) -> None:
        if auto_build:
            lib_path = build_filter_shared_library(force=force_rebuild)
        else:
            lib_path = Path(__file__).resolve().parent / "build" / "firm_filter.dll"

        self._lib = CDLL(str(lib_path))
        self._bind_signatures()

    def _bind_signatures(self) -> None:
        lib = self._lib

        lib.setup_reset.argtypes = []
        lib.setup_reset.restype = c_int

        lib.setup_set_orientation_variant.argtypes = [c_int]
        lib.setup_set_orientation_variant.restype = c_int

        lib.setup_set_orientation_matrices.argtypes = [POINTER(c_float), POINTER(c_float)]
        lib.setup_set_orientation_matrices.restype = c_int

        lib.setup_accumulate_sample.argtypes = [
            c_float,
            c_float,
            c_float,
            c_float,
            c_float,
            c_float,
            c_float,
        ]
        lib.setup_accumulate_sample.restype = c_int

        lib.setup_initialize.argtypes = []
        lib.setup_initialize.restype = c_int

        lib.setup_initialize_default.argtypes = [c_uint32]
        lib.setup_initialize_default.restype = c_int

        lib.setup_step.argtypes = [
            c_float,
            c_float,
            c_float,
            c_float,
            c_float,
            c_float,
            c_float,
            c_float,
            c_float,
            c_float,
            c_float,
        ]
        lib.setup_step.restype = c_int

        lib.setup_get_state.argtypes = [
            POINTER(c_float),
            POINTER(c_float),
            POINTER(c_float),
            POINTER(c_float),
            POINTER(c_float),
            POINTER(c_float),
        ]
        lib.setup_get_state.restype = c_int

        lib.setup_is_initialized.argtypes = []
        lib.setup_is_initialized.restype = c_int

    @staticmethod
    def _status_to_message(status: int, context: str) -> str:
        mapping = {
            STATUS_INVALID_ARGUMENT: "invalid argument",
            STATUS_NOT_INITIALIZED: "filter not initialized",
            STATUS_INIT_FAILED: "initialization failed",
            STATUS_INSUFFICIENT_ACCUMULATION: "insufficient startup accumulation",
        }
        reason = mapping.get(status, f"unknown status={status}")
        return f"{context}: {reason}"

    def _check_status(self, status: int, context: str) -> None:
        if status != STATUS_OK:
            raise FilterHarnessError(self._status_to_message(status, context))

    def reset(self) -> None:
        """Reset host-side filter state and accumulation buffers."""
        self._check_status(self._lib.setup_reset(), "setup_reset")

    def set_orientation_variant(self, variant: int = ORIENTATION_V2) -> None:
        """Select built-in orientation mapping constants (v1 or v2)."""
        self._check_status(
            self._lib.setup_set_orientation_variant(c_int(variant)),
            "setup_set_orientation_variant",
        )

    def set_orientation_matrices(
        self,
        imu_to_board: list[float],
        mag_to_board: list[float],
    ) -> None:
        """Inject custom 3x3 orientation matrices for IMU and magnetometer."""
        if len(imu_to_board) != 9 or len(mag_to_board) != 9:
            raise FilterHarnessError("orientation matrices must have exactly 9 floats each")

        matrix_t = c_float * 9
        imu = matrix_t(*imu_to_board)
        mag = matrix_t(*mag_to_board)

        self._check_status(
            self._lib.setup_set_orientation_matrices(imu, mag),
            "setup_set_orientation_matrices",
        )

    def accumulate_sample(
        self,
        pressure_pa: float,
        accel_x_g: float,
        accel_y_g: float,
        accel_z_g: float,
        mag_x_ut: float,
        mag_y_ut: float,
        mag_z_ut: float,
    ) -> None:
        """Feed one startup sample into ESKF accumulation buffers."""
        self._check_status(
            self._lib.setup_accumulate_sample(
                c_float(pressure_pa),
                c_float(accel_x_g),
                c_float(accel_y_g),
                c_float(accel_z_g),
                c_float(mag_x_ut),
                c_float(mag_y_ut),
                c_float(mag_z_ut),
            ),
            "setup_accumulate_sample",
        )

    def initialize(self) -> None:
        """Initialize the filter from previously accumulated startup samples."""
        self._check_status(self._lib.setup_initialize(), "setup_initialize")

    def initialize_default(self, sample_count: int = 64) -> None:
        """Initialize using built-in stationary sample defaults."""
        self._check_status(
            self._lib.setup_initialize_default(c_uint32(sample_count)),
            "setup_initialize_default",
        )

    def step(
        self,
        dt_s: float,
        pressure_pa: float,
        accel_x_g: float,
        accel_y_g: float,
        accel_z_g: float,
        gyro_x_dps: float,
        gyro_y_dps: float,
        gyro_z_dps: float,
        mag_x_ut: float,
        mag_y_ut: float,
        mag_z_ut: float,
    ) -> None:
        """Run one predict/update cycle with the provided sensor sample."""
        self._check_status(
            self._lib.setup_step(
                c_float(dt_s),
                c_float(pressure_pa),
                c_float(accel_x_g),
                c_float(accel_y_g),
                c_float(accel_z_g),
                c_float(gyro_x_dps),
                c_float(gyro_y_dps),
                c_float(gyro_z_dps),
                c_float(mag_x_ut),
                c_float(mag_y_ut),
                c_float(mag_z_ut),
            ),
            "setup_step",
        )

    def get_state(self) -> tuple[float, float, tuple[float, float, float, float]]:
        """Return altitude, vertical velocity, and quaternion (w, x, y, z)."""
        altitude = c_float()
        velocity = c_float()
        qw = c_float()
        qx = c_float()
        qy = c_float()
        qz = c_float()

        self._check_status(
            self._lib.setup_get_state(
                byref(altitude),
                byref(velocity),
                byref(qw),
                byref(qx),
                byref(qy),
                byref(qz),
            ),
            "setup_get_state",
        )

        return altitude.value, velocity.value, (qw.value, qx.value, qy.value, qz.value)

    def is_initialized(self) -> bool:
        """Return True if the underlying C filter is initialized."""
        return bool(self._lib.setup_is_initialized())
