"""Small manager fixture used by ESKF tests."""

from __future__ import annotations

import struct
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pathlib import Path


def make_archive(root: Path) -> Path:
    """Create an empty manager root."""
    (root / "launches").mkdir(parents=True)
    (root / "archive.yaml").write_text(
        "schema_version: 1\nlaunches_directory: launches\n", encoding="utf-8"
    )
    return root


def write_log(path: Path, *, samples: int) -> Path:
    """Write a compact v1.4 log with all required sensor streams."""
    calibration = [
        value
        for _ in range(4)
        for value in (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    ]
    header = (
        b"FIRM LOG v1.4\n"
        + (123).to_bytes(8, "little")
        + b"fixture".ljust(32, b"\0")
        + bytes([1, 0, 0, 0])
        + b"v2.2.0".ljust(8, b"\0")
        + struct.pack("<H", 100)
        + struct.pack("<48f", *calibration)
    )
    imu = bytes([0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    magnetometer = bytes([128, 0, 128, 0, 128, 0, 0])
    with path.open("wb") as handle:
        handle.write(header)
        for index in range(samples):
            timestamp = index * 1_680_000
            barometer = (20 * 65536).to_bytes(3, "little") + int((101325 - index) * 64).to_bytes(
                3, "little"
            )
            for identifier, payload in (
                (b"B", barometer),
                (b"I", imu),
                (b"M", magnetometer),
                (b"A", bytes(6)),
            ):
                handle.write(identifier + timestamp.to_bytes(4, "little") + payload)
        handle.write(bytes(128))
    return path
