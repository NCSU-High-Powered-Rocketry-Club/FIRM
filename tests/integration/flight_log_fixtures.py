"""Small binary fixtures shared by flight-manager integration tests."""

from __future__ import annotations

import math
import struct
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pathlib import Path


def identity_calibration(count: int = 4) -> list[float]:
    """Return packed identity blocks for a requested sensor count."""
    block = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    return block * count


def _imu_payload(accel: tuple[int, int, int], gyro: tuple[int, int, int]) -> bytes:
    result = bytearray(15)
    for axis, value in enumerate(accel):
        encoded = value & 0xFFFFF
        result[2 * axis] = (encoded >> 12) & 0xFF
        result[2 * axis + 1] = (encoded >> 4) & 0xFF
        result[12 + axis] |= (encoded & 0x0F) << 4
    for axis, value in enumerate(gyro):
        encoded = value & 0xFFFFF
        result[6 + 2 * axis] = (encoded >> 12) & 0xFF
        result[7 + 2 * axis] = (encoded >> 4) & 0xFF
        result[12 + axis] |= encoded & 0x0F
    return bytes(result)


def _mag_payload(values: tuple[int, int, int]) -> bytes:
    result = bytearray(7)
    for axis, value in enumerate(values):
        result[2 * axis] = (value >> 10) & 0xFF
        result[2 * axis + 1] = (value >> 2) & 0xFF
    result[6] = ((values[0] & 3) << 6) | ((values[1] & 3) << 4) | (values[2] & 0x0C)
    return bytes(result)


def _header(version: str) -> bytes:
    text = f"FIRM LOG v{version}\n".encode()
    scales = struct.pack("<6f", 65536.0, 64.0, 16384.0, 131.072, 163.84, 10.24)
    if version == "1.0":
        return text + scales[:20]
    uid = (123).to_bytes(8, "little")
    if version == "1.1":
        return (
            text
            + uid
            + b"fixture".ljust(33, b"\0")
            + bytes([1, 0])
            + bytes(5)
            + struct.pack("<36f", *identity_calibration(3))
            + scales[:20]
        )
    common = (
        text
        + uid
        + b"fixture".ljust(32, b"\0")
        + bytes([1, 0, 0, 0])
        + b"v2.2.0".ljust(8, b"\0")
        + struct.pack("<H", 100)
    )
    if version == "1.2":
        return common + bytes(2) + struct.pack("<36f", *identity_calibration(3)) + scales[:20]
    if version == "1.3":
        return common + bytes(2) + struct.pack("<48f", *identity_calibration()) + scales
    if version == "1.4":
        return common + struct.pack("<48f", *identity_calibration())
    raise ValueError(version)


def write_log(path: Path, *, version: str = "1.4", samples: int = 40) -> Path:
    """Write a compact historical log fixture."""
    timestamp_bytes = 3 if version in {"1.0", "1.1"} else 4
    byteorder = "big" if version == "1.0" else "little"
    with path.open("wb") as handle:
        handle.write(_header(version))
        for index in range(samples):
            timestamp = (index * 1_680_000) % (1 << (timestamp_bytes * 8))
            packets = (
                (
                    b"B",
                    int(20.0 * 65536).to_bytes(3, "little")
                    + int((101325.0 - index) * 64).to_bytes(3, "little"),
                ),
                (b"I", _imu_payload((0, 0, 16384), (0, 0, 0))),
                (
                    b"M",
                    _mag_payload((131072 + index * 4, 131072 + index * 4, 131072 + index * 4)),
                ),
                (b"A", bytes(6)),
            )
            for identifier, payload in packets:
                handle.write(identifier)
                handle.write(timestamp.to_bytes(timestamp_bytes, byteorder))
                handle.write(payload)
        handle.write(bytes(128))
    return path


def inject_v10_magnetometer_anomaly(path: Path, sample_index: int) -> None:
    """Replace one v1.0 magnetometer payload with a representative spike."""
    header_size = len(_header("1.0"))
    bytes_per_sample = (1 + 3 + 6) + (1 + 3 + 15) + (1 + 3 + 7) + (1 + 3 + 6)
    magnetometer_payload_offset = header_size + sample_index * bytes_per_sample + 10 + 19 + 4
    with path.open("r+b") as handle:
        handle.seek(magnetometer_payload_offset)
        handle.write(_mag_payload((200000, 200000, 200000)))


def write_phase_log(path: Path, *, samples: int = 800) -> Path:
    """Write a v1.4 fixture with corroborated liftoff, apogee, and landing phases."""
    with path.open("wb") as handle:
        handle.write(_header("1.4"))
        for index in range(samples):
            time_s = index / 100.0
            if time_s < 1.0:
                altitude_m, acceleration_g = 0.0, 1.0
            elif time_s < 1.5:
                altitude_m, acceleration_g = (time_s - 1.0) * 40.0, 2.0
            elif time_s < 3.0:
                altitude_m, acceleration_g = 20.0 + (time_s - 1.5) * (80.0 / 1.5), 1.0
            elif time_s < 5.0:
                altitude_m, acceleration_g = 100.0 - (time_s - 3.0) * 50.0, 1.0
            else:
                altitude_m, acceleration_g = 0.0, 1.0
            pressure_pa = 101325.0 * math.pow(1.0 - altitude_m / 44330.0, 1 / 0.190294957)
            timestamp = index * 1_680_000
            packets = (
                (
                    b"B",
                    int(20.0 * 65536).to_bytes(3, "little")
                    + int(pressure_pa * 64).to_bytes(3, "little"),
                ),
                (b"I", _imu_payload((0, 0, int(acceleration_g * 16384)), (0, 0, 0))),
                (b"M", _mag_payload((131072, 131072, 131072))),
                (b"A", bytes(6)),
            )
            for identifier, payload in packets:
                handle.write(identifier + timestamp.to_bytes(4, "little") + payload)
        handle.write(bytes(128))
    return path


def make_archive(root: Path) -> Path:
    """Create an empty manager root."""
    (root / "launches").mkdir(parents=True)
    (root / "archive.yaml").write_text(
        "schema_version: 1\nlaunches_directory: launches\n", encoding="utf-8"
    )
    return root
