"""Single source of truth for historical FIRM binary log formats."""

from __future__ import annotations

import math
import struct
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO, Literal, cast

from .models import (
    CalibrationValues,
    EffectiveCalibration,
    LogHeader,
    LogPacket,
    TrimWindow,
    ValidationReport,
)

CPU_HZ = 168_000_000.0
HEADER_TEXT_SIZE = 14
LATEST_VERSION = "1.4"
HARDWARE_VERSIONS = ("old", "new")
TARGET_HARDWARE = "new"
PACKET_SIZES = {
    b"B"[0]: ("barometer", 6),
    b"I"[0]: ("imu", 15),
    b"M"[0]: ("magnetometer", 7),
    b"A"[0]: ("high_g", 6),
}


def normalize_hardware_version(value: str) -> str:
    """Validate a source-hardware selection used during canonical migration."""
    normalized = value.strip().lower()
    if normalized not in HARDWARE_VERSIONS:
        raise ValueError(f"hardware must be one of: {', '.join(HARDWARE_VERSIONS)}")
    return normalized


@dataclass(frozen=True)
class LogFormat:
    """Layout metadata for one on-disk log version."""

    version: str
    timestamp_bytes: int
    timestamp_byteorder: Literal["little", "big"]
    header_size: int
    calibration_sensors: int


FORMATS = {
    "1.0": LogFormat("1.0", 3, "big", 14 + 20, 0),
    "1.1": LogFormat("1.1", 3, "little", 14 + 8 + 33 + 2 + 5 + 144 + 20, 3),
    "1.2": LogFormat("1.2", 4, "little", 14 + 8 + 32 + 4 + 8 + 2 + 2 + 144 + 20, 3),
    "1.3": LogFormat("1.3", 4, "little", 14 + 8 + 32 + 4 + 8 + 2 + 2 + 192 + 24, 4),
    "1.4": LogFormat("1.4", 4, "little", 14 + 8 + 32 + 4 + 8 + 2 + 192, 4),
}


def _read_exact(handle: BinaryIO, length: int, field: str) -> bytes:
    data = handle.read(length)
    if len(data) != length:
        raise ValueError(f"unexpected EOF while reading {field}")
    return data


def _text(data: bytes) -> str:
    return data.rstrip(b"\0 ").decode("utf-8", errors="replace")


def _default_calibration() -> EffectiveCalibration:
    return EffectiveCalibration.identity()


def _unpack_calibration(data: bytes, count: int) -> EffectiveCalibration:
    result = _default_calibration().values.copy()
    names = ("accel", "gyro", "mag", "high_g")
    values = struct.unpack(f"<{count * 12}f", data)
    for index in range(count):
        block = values[index * 12 : (index + 1) * 12]
        result[names[index]] = CalibrationValues(tuple(block[:3]), tuple(block[3:]))  # type: ignore[arg-type]
    return EffectiveCalibration(result)


def read_header(handle: BinaryIO) -> LogHeader:
    """Read and normalize any supported FIRM header."""
    handle.seek(0)
    header_text = _read_exact(handle, HEADER_TEXT_SIZE, "header text")
    try:
        version = header_text.decode("ascii").strip().removeprefix("FIRM LOG v")
    except UnicodeDecodeError as error:
        raise ValueError("invalid FIRM log header") from error
    if version not in FORMATS or header_text != f"FIRM LOG v{version}\n".encode():
        raise ValueError(f"unsupported FIRM log header {header_text!r}")
    spec = FORMATS[version]

    if version == "1.0":
        _read_exact(handle, 20, "v1.0 scale factors")
        return LogHeader(
            version,
            int.from_bytes(b"1" * 8, "little"),
            "driver0",
            (True, False, False, False),
            "v1.0.0",
            100,
            _default_calibration(),
            spec.header_size,
        )

    uid = int.from_bytes(_read_exact(handle, 8, "device UID"), "little")
    if version == "1.1":
        name = _text(_read_exact(handle, 33, "device name"))
        raw_comms = _read_exact(handle, 2, "communications")
        comms = (bool(raw_comms[0]), bool(raw_comms[1]), False, False)
        _read_exact(handle, 5, "padding")
        calibration = _unpack_calibration(_read_exact(handle, 144, "calibration"), 3)
        _read_exact(handle, 20, "scale factors")
        return LogHeader(version, uid, name, comms, "v1.0.0", 100, calibration, spec.header_size)

    name = _text(_read_exact(handle, 32, "device name"))
    raw_comms = _read_exact(handle, 4, "communications")
    comms = (bool(raw_comms[0]), bool(raw_comms[1]), bool(raw_comms[2]), bool(raw_comms[3]))
    firmware = _text(_read_exact(handle, 8, "firmware version"))
    frequency = struct.unpack("<H", _read_exact(handle, 2, "frequency"))[0]
    if version in {"1.2", "1.3"}:
        _read_exact(handle, 2, "padding")
    calibration = _unpack_calibration(
        _read_exact(handle, spec.calibration_sensors * 48, "calibration"),
        spec.calibration_sensors,
    )
    if version == "1.2":
        _read_exact(handle, 20, "scale factors")
    elif version == "1.3":
        _read_exact(handle, 24, "scale factors")
    return LogHeader(version, uid, name, comms, firmware, frequency, calibration, spec.header_size)


def _mag_bins(payload: bytes) -> tuple[int, int, int]:
    return (
        (payload[0] << 10) | (payload[1] << 2) | (payload[6] >> 6),
        (payload[2] << 10) | (payload[3] << 2) | ((payload[6] & 0x30) >> 4),
        (payload[4] << 10) | (payload[5] << 2) | (payload[6] & 0x0C),
    )


def _max_abs_diff(left: tuple[int, int, int], right: tuple[int, int, int]) -> int:
    return max(abs(a - b) for a, b in zip(left, right, strict=True))


class LogReader:
    """Streaming reader and validator for a historical FIRM log."""

    def __init__(
        self,
        path: str | Path,
        *,
        time_origin_ticks: int | None = None,
        first_unwrapped_ticks: int | None = None,
    ):
        self.path = Path(path).resolve()
        with self.path.open("rb") as handle:
            self.header = read_header(handle)
        self.format = FORMATS[self.header.version]
        self._time_origin_ticks = time_origin_ticks
        self._first_unwrapped_ticks = first_unwrapped_ticks
        self._v10_anomaly_start: int | bool | None = False

    def _raw_packets(self) -> Iterator[tuple[int, int, bytes, int]]:
        with self.path.open("rb") as handle:
            handle.seek(self.header.header_size)
            zeroes = 0
            packet_index = 0
            while True:
                identifier_data = handle.read(1)
                if not identifier_data:
                    return
                identifier = identifier_data[0]
                if identifier == 0:
                    zeroes += 1
                    if zeroes > 20:
                        return
                    continue
                zeroes = 0
                metadata = PACKET_SIZES.get(identifier)
                if metadata is None:
                    return
                timestamp_data = handle.read(self.format.timestamp_bytes)
                if len(timestamp_data) != self.format.timestamp_bytes:
                    return
                payload = handle.read(metadata[1])
                if len(payload) != metadata[1]:
                    return
                timestamp = int.from_bytes(timestamp_data, self.format.timestamp_byteorder)
                yield identifier, timestamp, payload, handle.tell()
                packet_index += 1

    def _detect_v10_anomaly(self) -> int | None:
        bins: list[tuple[int, int, int]] = []
        indices: list[int] = []
        mag_index = 0
        for identifier, _timestamp, payload, _offset in self._raw_packets():
            if identifier != ord("M"):
                continue
            bins.append(_mag_bins(payload))
            indices.append(mag_index)
            mag_index += 1
            if len(bins) < 3:
                continue
            previous, current, following = bins[-3:]
            if (
                _max_abs_diff(previous, current) >= 10_000
                and _max_abs_diff(current, following) >= 10_000
                and _max_abs_diff(previous, following) <= 5_000
            ):
                return indices[-2] if indices[-2] > 0 else None
        return None

    def iter_packets(self) -> Iterator[LogPacket]:
        """Yield corrected, timestamp-unwrapped packets until padding or invalid trailing data."""
        if self.header.version == "1.0" and self._v10_anomaly_start is False:
            self._v10_anomaly_start = self._detect_v10_anomaly()
        anomaly_start = self._v10_anomaly_start if type(self._v10_anomaly_start) is int else None
        modulus = 1 << (8 * self.format.timestamp_bytes)
        previous_timestamp: int | None = None
        unwrapped = 0
        first_unwrapped: int | None = None
        mag_index = 0
        previous_mag: bytes | None = None
        for packet_index, (identifier, timestamp, raw_payload, _offset) in enumerate(
            self._raw_packets()
        ):
            payload = raw_payload
            if previous_timestamp is None:
                unwrapped = (
                    self._first_unwrapped_ticks
                    if self._first_unwrapped_ticks is not None
                    else timestamp
                )
                first_unwrapped = (
                    self._time_origin_ticks if self._time_origin_ticks is not None else unwrapped
                )
            else:
                unwrapped += (timestamp - previous_timestamp) % modulus
            previous_timestamp = timestamp
            if identifier == ord("M"):
                if (
                    anomaly_start is not None
                    and mag_index >= anomaly_start
                    and (mag_index - anomaly_start) % 11 == 0
                    and previous_mag is not None
                ):
                    payload = previous_mag
                else:
                    previous_mag = payload
                mag_index += 1
            if first_unwrapped is None:
                raise RuntimeError("packet timestamp initialization failed")
            sensor = PACKET_SIZES[identifier][0]
            yield LogPacket(
                packet_index,
                sensor,
                identifier,
                timestamp,
                unwrapped,
                (unwrapped - first_unwrapped) / CPU_HZ,
                payload,
            )

    def validate(self) -> ValidationReport:
        report = ValidationReport(self.path, version=self.header.version)
        try:
            for packet in self.iter_packets():
                report.packet_count += 1
                report.duration_seconds = packet.timestamp_s
            for _identifier, _timestamp, _payload, offset in self._raw_packets():
                report.last_complete_offset = offset
            if report.packet_count == 0:
                report.errors.append("log contains no complete sensor packets")
            if self.path.stat().st_size > report.last_complete_offset:
                trailing = self.path.stat().st_size - report.last_complete_offset
                report.warnings.append(f"{trailing} trailing bytes will be removed")
        except (OSError, ValueError) as error:
            report.errors.append(str(error))
        return report


def _calibration_bytes(calibration: EffectiveCalibration) -> bytes:
    floats: list[float] = []
    for name in ("accel", "gyro", "mag", "high_g"):
        values = calibration.values[name]
        floats.extend(values.offset)
        floats.extend(values.matrix)
    return struct.pack("<48f", *floats)


_OLD_TO_NEW_IMU_ROTATION = (
    (0.0, 1.0, 0.0),
    (-1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0),
)

Matrix3 = tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]


def _matrix_multiply(
    left: Matrix3,
    right: Matrix3,
) -> Matrix3:
    return cast(
        "Matrix3",
        tuple(
            tuple(
                sum(left[row][inner] * right[inner][column] for inner in range(3))
                for column in range(3)
            )
            for row in range(3)
        ),
    )


def _rotate_calibration_values(values: CalibrationValues) -> CalibrationValues:
    # Calibration uses row vectors: corrected = (raw - offset) * matrix. Since
    # raw_new = raw_old * Q^T, the equivalent new-frame matrix is Q * matrix * Q^T.
    rotation = _OLD_TO_NEW_IMU_ROTATION
    rotation_transpose = cast("Matrix3", tuple(zip(*rotation, strict=True)))
    matrix = cast(
        "Matrix3",
        tuple(tuple(values.matrix[row * 3 + column] for column in range(3)) for row in range(3)),
    )
    transformed_matrix = _matrix_multiply(_matrix_multiply(rotation, matrix), rotation_transpose)
    offset = values.offset
    transformed_offset = (offset[1], -offset[0], offset[2])
    return CalibrationValues(
        transformed_offset,
        cast(
            "tuple[float, float, float, float, float, float, float, float, float]",
            tuple(value for row in transformed_matrix for value in row),
        ),
    )


def calibration_for_hardware(
    calibration: EffectiveCalibration, source_hardware: str
) -> EffectiveCalibration:
    """Return calibration expressed in the current hardware's IMU coordinate frame."""
    hardware = normalize_hardware_version(source_hardware)
    if hardware == TARGET_HARDWARE:
        return calibration
    values = calibration.values.copy()
    values["accel"] = _rotate_calibration_values(values["accel"])
    values["gyro"] = _rotate_calibration_values(values["gyro"])
    return EffectiveCalibration(values)


def _unpack_imu_payload(payload: bytes) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
    if len(payload) != 15:
        raise ValueError(f"IMU payload must be 15 bytes, got {len(payload)}")
    accel = (
        _signed((payload[0] << 12) | (payload[1] << 4) | (payload[12] >> 4), 20),
        _signed((payload[2] << 12) | (payload[3] << 4) | (payload[13] >> 4), 20),
        _signed((payload[4] << 12) | (payload[5] << 4) | (payload[14] >> 4), 20),
    )
    gyro = (
        _signed((payload[6] << 12) | (payload[7] << 4) | (payload[12] & 0x0F), 20),
        _signed((payload[8] << 12) | (payload[9] << 4) | (payload[13] & 0x0F), 20),
        _signed((payload[10] << 12) | (payload[11] << 4) | (payload[14] & 0x0F), 20),
    )
    return accel, gyro


def _pack_imu_payload(accel: tuple[int, int, int], gyro: tuple[int, int, int]) -> bytes:
    payload = bytearray(15)
    for axis, raw_value in enumerate(accel):
        value = raw_value & 0xFFFFF
        payload[axis * 2] = (value >> 12) & 0xFF
        payload[axis * 2 + 1] = (value >> 4) & 0xFF
        payload[12 + axis] = (value & 0x0F) << 4
    for axis, raw_value in enumerate(gyro):
        value = raw_value & 0xFFFFF
        payload[6 + axis * 2] = (value >> 12) & 0xFF
        payload[7 + axis * 2] = (value >> 4) & 0xFF
        payload[12 + axis] |= value & 0x0F
    return bytes(payload)


def _negate_20_bit(value: int) -> int:
    # Positive two's-complement range is one count smaller; clip the unique
    # unrepresentable rotation of -524288 to +524287.
    return (1 << 19) - 1 if value == -(1 << 19) else -value


def rotate_old_hardware_imu_payload(payload: bytes) -> bytes:
    """Rotate accel and gyro raw bins from old to new axes: `(x,y,z) -> (y,-x,z)`."""
    accel, gyro = _unpack_imu_payload(payload)
    rotated_accel = (accel[1], _negate_20_bit(accel[0]), accel[2])
    rotated_gyro = (gyro[1], _negate_20_bit(gyro[0]), gyro[2])
    return _pack_imu_payload(rotated_accel, rotated_gyro)


def write_v14_header(
    handle: BinaryIO, source: LogHeader, calibration: EffectiveCalibration
) -> None:
    """Write an explicit v1.4 header without serializing a host-language struct."""
    handle.write(b"FIRM LOG v1.4\n")
    handle.write(source.device_uid.to_bytes(8, "little"))
    handle.write(source.device_name.encode("utf-8", errors="replace")[:32].ljust(32, b"\0"))
    handle.write(bytes(int(value) for value in source.communications))
    handle.write(source.firmware_version.encode("ascii", errors="replace")[:8].ljust(8, b"\0"))
    handle.write(struct.pack("<H", source.frequency_hz))
    handle.write(_calibration_bytes(calibration))


class LogWriter:
    """Current-version log writer used by migration and trimming."""

    target_version = LATEST_VERSION

    @staticmethod
    def migrate(
        source: str | Path,
        destination: str | Path,
        calibration: EffectiveCalibration,
        trim_window: TrimWindow | None = None,
        *,
        source_hardware: str,
    ) -> int:
        reader = LogReader(source)
        hardware = normalize_hardware_version(source_hardware)
        output_calibration = calibration_for_hardware(calibration, hardware)
        destination = Path(destination)
        packet_count = 0
        with destination.open("wb") as handle:
            write_v14_header(handle, reader.header, output_calibration)
            for packet in reader.iter_packets():
                if trim_window is not None and not (
                    trim_window.start_s <= packet.timestamp_s <= trim_window.end_s
                ):
                    continue
                handle.write(bytes([packet.identifier]))
                handle.write((packet.unwrapped_timestamp_ticks & 0xFFFFFFFF).to_bytes(4, "little"))
                payload = (
                    rotate_old_hardware_imu_payload(packet.payload)
                    if hardware == "old" and packet.sensor == "imu"
                    else packet.payload
                )
                handle.write(payload)
                packet_count += 1
        if packet_count == 0:
            destination.unlink(missing_ok=True)
            raise ValueError("selected interval contains no complete packets")
        return packet_count


def _signed(value: int, bits: int) -> int:
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


def apply_calibration(
    vector: tuple[float, float, float], calibration: CalibrationValues
) -> tuple[float, float, float]:
    """Apply the firmware's `(raw - offset) * matrix` convention."""
    adjusted = tuple(
        value - offset for value, offset in zip(vector, calibration.offset, strict=True)
    )
    matrix = calibration.matrix
    return (
        adjusted[0] * matrix[0] + adjusted[1] * matrix[3] + adjusted[2] * matrix[6],
        adjusted[0] * matrix[1] + adjusted[1] * matrix[4] + adjusted[2] * matrix[7],
        adjusted[0] * matrix[2] + adjusted[1] * matrix[5] + adjusted[2] * matrix[8],
    )


def decode_packet(packet: LogPacket, calibration: EffectiveCalibration) -> dict[str, int | float]:
    """Decode raw payload fields and corrected engineering values."""
    base: dict[str, int | float] = {
        "packet_index": packet.packet_index,
        "timestamp_ticks": packet.unwrapped_timestamp_ticks,
        "timestamp_s": packet.timestamp_s,
    }
    data = packet.payload
    if packet.sensor == "barometer":
        raw_temperature = int.from_bytes(data[:3], "little")
        raw_pressure = int.from_bytes(data[3:], "little")
        base.update(
            raw_temperature=raw_temperature,
            raw_pressure=raw_pressure,
            uncalibrated_temperature_c=raw_temperature / 65536.0,
            uncalibrated_pressure_pa=raw_pressure / 64.0,
            temperature_c=raw_temperature / 65536.0,
            pressure_pa=raw_pressure / 64.0,
        )
        return base
    if packet.sensor == "imu":
        raw_accel = (
            _signed((data[0] << 12) | (data[1] << 4) | (data[12] >> 4), 20),
            _signed((data[2] << 12) | (data[3] << 4) | (data[13] >> 4), 20),
            _signed((data[4] << 12) | (data[5] << 4) | (data[14] >> 4), 20),
        )
        raw_gyro = (
            _signed((data[6] << 12) | (data[7] << 4) | (data[12] & 0x0F), 20),
            _signed((data[8] << 12) | (data[9] << 4) | (data[13] & 0x0F), 20),
            _signed((data[10] << 12) | (data[11] << 4) | (data[14] & 0x0F), 20),
        )
        accel = (raw_accel[0] / 16384.0, raw_accel[1] / 16384.0, raw_accel[2] / 16384.0)
        gyro = (raw_gyro[0] / 131.072, raw_gyro[1] / 131.072, raw_gyro[2] / 131.072)
        corrected_accel = apply_calibration(accel, calibration.values["accel"])
        corrected_gyro = apply_calibration(gyro, calibration.values["gyro"])
        for axis, raw, uncalibrated, corrected in zip(
            "xyz", raw_accel, accel, corrected_accel, strict=True
        ):
            base[f"raw_accel_{axis}"] = raw
            base[f"uncalibrated_accel_{axis}_g"] = uncalibrated
            base[f"imu_accel_{axis}_g"] = corrected
        for axis, raw, uncalibrated, corrected in zip(
            "xyz", raw_gyro, gyro, corrected_gyro, strict=True
        ):
            base[f"raw_gyro_{axis}"] = raw
            base[f"uncalibrated_gyro_{axis}_dps"] = uncalibrated
            base[f"imu_gyro_{axis}_dps"] = corrected
        return base
    if packet.sensor == "magnetometer":
        raw = _mag_bins(data)
        uncalibrated = (
            (raw[0] - 131072) / 163.84,
            (raw[1] - 131072) / 163.84,
            (raw[2] - 131072) / 163.84,
        )
        corrected = apply_calibration(uncalibrated, calibration.values["mag"])
        for axis, raw_value, uncalibrated_value, corrected_value in zip(
            "xyz", raw, uncalibrated, corrected, strict=True
        ):
            base[f"raw_mag_{axis}"] = raw_value
            base[f"uncalibrated_mag_{axis}_ut"] = uncalibrated_value
            base[f"mag_{axis}_ut"] = corrected_value
        return base
    if packet.sensor == "high_g":
        raw = (
            _signed((data[0] << 4) | (data[1] >> 4), 12),
            _signed((data[2] << 4) | (data[3] >> 4), 12),
            _signed((data[4] << 4) | (data[5] >> 4), 12),
        )
        uncalibrated = (raw[0] / 10.24, raw[1] / 10.24, raw[2] / 10.24)
        corrected = apply_calibration(uncalibrated, calibration.values["high_g"])
        for axis, raw_value, uncalibrated_value, corrected_value in zip(
            "xyz", raw, uncalibrated, corrected, strict=True
        ):
            base[f"raw_accel_{axis}"] = raw_value
            base[f"uncalibrated_accel_{axis}_g"] = uncalibrated_value
            base[f"high_g_accel_{axis}_g"] = corrected_value
        return base
    raise ValueError(f"cannot decode unknown sensor {packet.sensor!r}")


def vector_norm(values: tuple[float, float, float]) -> float:
    return math.sqrt(sum(value * value for value in values))
