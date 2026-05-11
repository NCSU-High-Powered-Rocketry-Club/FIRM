from __future__ import annotations

from enum import IntEnum
from types import TracebackType
from typing import ClassVar, Optional, Type

__version__: str

class DeviceProtocol(IntEnum):
    """Enum of the supported device communication protocols."""

    USB = 1
    UART = 2
    I2C = 3
    SPI = 4

class DeviceInfo:
    """Represents information about the FIRM device."""

    firmware_version: str
    id: int

class DeviceConfig:
    """Represents the configuration of the FIRM device."""

    name: str
    frequency: int
    protocol: DeviceProtocol

class CalibrationValues:
    """Represents the calibration values for the FIRM device."""

    imu_accelerometer_offsets: tuple[float, float, float]
    imu_accelerometer_scale_matrix: tuple[
        float, float, float, float, float, float, float, float, float
    ]
    imu_gyroscope_offsets: tuple[float, float, float]
    imu_gyroscope_scale_matrix: tuple[
        float, float, float, float, float, float, float, float, float
    ]
    magnetometer_offsets: tuple[float, float, float]
    magnetometer_scale_matrix: tuple[
        float, float, float, float, float, float, float, float, float
    ]

class FIRMDataPacket:
    """Represents a data packet received from the FIRM device."""

    __struct_fields__: ClassVar[list[str]]
    """List of all field names in this packet."""

    timestamp_seconds: float
    """Timestamp of the data packet in seconds."""

    temperature_celsius: float
    """Ambient temperature measured in degrees Celsius."""

    pressure_pascals: float
    """Atmospheric pressure measured in Pascals."""

    raw_acceleration_x_gs: float
    """Raw accelerometer reading for X-axis in Gs."""
    raw_acceleration_y_gs: float
    """Raw accelerometer reading for Y-axis in Gs."""
    raw_acceleration_z_gs: float
    """Raw accelerometer reading for Z-axis in Gs."""

    raw_rotated_acceleration_x_gs: float
    """Raw acceleration X rotated into world frame in Gs."""
    raw_rotated_acceleration_y_gs: float
    """Raw acceleration Y rotated into world frame in Gs."""
    raw_rotated_acceleration_z_gs: float
    """Raw acceleration Z rotated into world frame in Gs."""

    est_tilt_angle_degrees: float
    """Total tilt angle from +Z based on raw acceleration in degrees."""

    raw_angular_rate_x_deg_per_s: float
    """Raw gyroscope reading for X-axis in degrees per second."""
    raw_angular_rate_y_deg_per_s: float
    """Raw gyroscope reading for Y-axis in degrees per second."""
    raw_angular_rate_z_deg_per_s: float
    """Raw gyroscope reading for Z-axis in degrees per second."""

    magnetic_field_x_microteslas: float
    """Magnetometer reading for X-axis in micro-Teslas."""
    magnetic_field_y_microteslas: float
    """Magnetometer reading for Y-axis in micro-Teslas."""
    magnetic_field_z_microteslas: float
    """Magnetometer reading for Z-axis in micro-Teslas."""

    est_position_z_meters: float
    """Estimated position along the Z-axis in meters."""

    est_velocity_z_meters_per_s: float
    """Estimated velocity along the Z-axis in meters per second."""

    est_mach_number: float
    """Estimated Mach number from velocity magnitude and ambient temperature."""

    est_quaternion_w: float
    """Estimated orientation quaternion scalar component (W)."""
    est_quaternion_x: float
    """Estimated orientation quaternion vector component (X)."""
    est_quaternion_y: float
    """Estimated orientation quaternion vector component (Y)."""
    est_quaternion_z: float
    """Estimated orientation quaternion vector component (Z)."""

    def __init__(
        self,
        timestamp_seconds: float,
        temperature_celsius: float,
        pressure_pascals: float,
        raw_acceleration_x_gs: float,
        raw_acceleration_y_gs: float,
        raw_acceleration_z_gs: float,
        raw_angular_rate_x_deg_per_s: float,
        raw_angular_rate_y_deg_per_s: float,
        raw_angular_rate_z_deg_per_s: float,
        magnetic_field_x_microteslas: float,
        magnetic_field_y_microteslas: float,
        magnetic_field_z_microteslas: float,
        est_position_z_meters: float,
        est_velocity_z_meters_per_s: float,
        est_quaternion_w: float,
        est_quaternion_x: float,
        est_quaternion_y: float,
        est_quaternion_z: float,
    ) -> None:
        """
        Initialize a new immutable FIRMDataPacket. All fields are required.
        """
        ...

    @staticmethod
    def default_zero() -> "FIRMDataPacket":
        """
        Creates a packet with zeroed values (and identity quaternion).
        """
        ...

    def as_dict(self) -> dict[str, float]:
        """
        Converts the packet to a standard Python dictionary.
        Returns a copy of the data.
        """
        ...

class MockDeviceHandle:
    """Handle for controlling an in-process mock device."""

    def inject_response(self, identifier: int, payload: bytes | bytearray) -> None: ...
    """Inject a response packet (identifier + raw payload bytes) into the mock device."""

    def wait_for_command_identifier(self, timeout_seconds: float) -> int | None: ...
    """Wait up to timeout_seconds for a command to be observed; returns its identifier or None."""

class FIRMClient:
    """Client for communicating with the FIRM device.

    Args:
        port_name: The name of the serial port to connect to.
        baud_rate: The baud rate for the serial connection. Default is 2,000,000.
        timeout: Read timeout used when get_data_packets(block=True). Default is 0.1 seconds.
    """

    def __init__(
        self, port_name: str, baud_rate: int = 2_000_000, timeout: float = 0.1
    ) -> None: ...
    @staticmethod
    def new_mock(timeout: float = 0.1) -> tuple[FIRMClient, MockDeviceHandle]: ...
    """Create a client + mock device pair for testing."""

    def start(self) -> None: ...
    """Start the background reader thread."""

    def stop(self) -> None: ...
    """Stop the background reader thread and close the serial port."""

    def get_data_packets(self, block: bool = False) -> list[FIRMDataPacket]: ...
    """Retrieve currently-available data packets.

    Args:
        block: If True, blocks up to `timeout` (from __init__) waiting for packets.
    """

    def get_device_info(self, timeout_seconds: float = 5.0) -> DeviceInfo | None: ...
    """Request device info and wait up to timeout_seconds."""

    def get_device_config(
        self, timeout_seconds: float = 5.0
    ) -> DeviceConfig | None: ...
    """Request device configuration and wait up to timeout_seconds."""

    def set_device_config(
        self,
        name: str,
        frequency: int,
        protocol: DeviceProtocol,
        timeout_seconds: float = 5.0,
    ) -> bool: ...
    """Set device config and wait up to timeout_seconds for acknowledgement."""

    def set_magnetometer_calibration(
        self,
        offsets: tuple[float, float, float],
        scale_matrix: tuple[
            float, float, float, float, float, float, float, float, float
        ],
        timeout_seconds: float = 5.0,
    ) -> bool: ...
    """Set magnetometer calibration and wait up to timeout_seconds for acknowledgement."""

    def set_imu_calibration(
        self,
        accel_offsets: tuple[float, float, float],
        accel_scale_matrix: tuple[
            float, float, float, float, float, float, float, float, float
        ],
        gyro_offsets: tuple[float, float, float],
        gyro_scale_matrix: tuple[
            float, float, float, float, float, float, float, float, float
        ],
        timeout_seconds: float = 5.0,
    ) -> bool: ...
    """Set IMU calibration and wait up to timeout_seconds for acknowledgement."""

    def get_calibration(
        self, timeout_seconds: float = 5.0
    ) -> CalibrationValues | None: ...
    """Request calibration values and wait up to timeout_seconds."""

    def cancel(self, timeout_seconds: float = 5.0) -> bool: ...
    """Send cancel and wait up to timeout_seconds for acknowledgement."""

    def reboot(self) -> None: ...
    """Send reboot command."""

    def start_mock_log_stream(
        self,
        log_path: str,
        realtime: bool = True,
        speed: float = 1.0,
        chunk_size: int = 8192,
        start_timeout_seconds: float = 5.0,
        cancel_on_finish: bool = True,
    ) -> None: ...
    """Start streaming a mock log file asynchronously in the background."""

    def is_mock_log_streaming(self) -> bool: ...
    """True if a mock log stream is currently running."""

    def stop_mock_log_stream(
        self, cancel_device: bool = True, join: bool = True
    ) -> int | None: ...
    """Stop the async mock log stream. Optionally cancel the device."""

    def run_and_apply_magnetometer_calibration(
        self,
        collection_duration_seconds: float,
        apply_timeout_seconds: float = 5.0,
    ) -> Optional[bool]: ...
    """Run magnetometer calibration procedure and sets the constants on the device."""

    def is_running(self) -> bool: ...
    """True if the client reader thread is running."""

    def __enter__(self) -> FIRMClient: ...
    """Context manager which calls start()."""

    def __exit__(
        self,
        exc_type: Type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None: ...
    """Context manager which calls stop()."""
