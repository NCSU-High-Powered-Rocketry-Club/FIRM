"""Reconstruct a FIRM v1.4 log from the four legacy decoder CSV files.

This is a recovery tool, not a general flight-data-manager input format. The CSVs preserve the
decoded raw sensor bins closely enough to recover packet payloads, but a reconstructed file is not
the byte-for-byte original acquisition file.
"""

from __future__ import annotations

import argparse
import csv
import heapq
import math
import os
import struct
import sys
import uuid
from collections.abc import Callable, Iterable, Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import cast

CPU_HZ = 168_000_000
CALIBRATION_LABELS = (
    "ICM45686 Acceleration Calibration",
    "ICM45686 Gyroscope Calibration",
    "MMC5983MA Magnetometer Calibration",
    "ADXL371 Acceleration Calibration",
)


@dataclass(frozen=True)
class Header:
    """Device metadata repeated in each legacy CSV preamble."""

    device_name: str
    uid: int
    firmware_version: str
    communications: tuple[bool, bool, bool, bool]
    frequency_hz: int
    calibration: tuple[tuple[float, ...], ...]


@dataclass(frozen=True)
class SensorSpec:
    """Expected CSV layout and corresponding binary packet encoder."""

    filename: str
    identifier: bytes
    columns: tuple[str, ...]
    pack: Callable[[dict[str, float], str], bytes]


def _boolean(value: str, field: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"true", "1", "yes"}:
        return True
    if normalized in {"false", "0", "no"}:
        return False
    raise ValueError(f"{field} must be True or False, got {value!r}")


def _numbers(row: list[str], count: int, field: str) -> tuple[float, ...]:
    values = tuple(float(value) for value in row[1:] if value.strip())
    if len(values) != count or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{field} must contain exactly {count} finite values")
    return values


def read_header(path: Path, expected_columns: tuple[str, ...]) -> Header:
    """Read and validate the legacy decoder preamble and data-column row."""
    fields: dict[str, object] = {}
    calibrations: dict[str, tuple[float, ...]] = {}
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.reader(handle)
        for line_number, row in enumerate(reader, start=1):
            if not row or not any(value.strip() for value in row):
                continue
            label = row[0].strip()
            if label.lower() == "timestamp":
                columns = tuple(value.strip() for value in row)
                if columns != expected_columns:
                    raise ValueError(
                        f"{path}:{line_number}: expected columns {expected_columns}, got {columns}"
                    )
                break
            if line_number == 1:
                if len(row) < 2:
                    raise ValueError(f"{path}: first row must contain device name and UID")
                fields["device_name"] = label
                fields["uid"] = int(row[1])
            elif label == "FIRM version:":
                fields["firmware_version"] = row[1].strip()
            elif label == "usb enabled:":
                fields["usb"] = _boolean(row[1], label)
            elif label == "uart enabled:":
                fields["uart"] = _boolean(row[1], label)
            elif label == "i2c enabled:":
                fields["i2c"] = _boolean(row[1], label)
            elif label == "spi enabled:":
                fields["spi"] = _boolean(row[1], label)
            elif label == "Transmit Frequency:":
                fields["frequency_hz"] = int(row[1])
            elif label in CALIBRATION_LABELS:
                calibrations[label] = _numbers(row, 12, label)
        else:
            raise ValueError(f"{path}: timestamp column header was not found")

    required = {
        "device_name",
        "uid",
        "firmware_version",
        "usb",
        "uart",
        "i2c",
        "spi",
        "frequency_hz",
    }
    missing = required - fields.keys()
    if missing:
        raise ValueError(f"{path}: missing header fields: {', '.join(sorted(missing))}")
    missing_calibration = set(CALIBRATION_LABELS) - calibrations.keys()
    if missing_calibration:
        raise ValueError(
            f"{path}: missing calibration rows: {', '.join(sorted(missing_calibration))}"
        )
    return Header(
        str(fields["device_name"]),
        cast("int", fields["uid"]),
        str(fields["firmware_version"]),
        (
            bool(fields["usb"]),
            bool(fields["uart"]),
            bool(fields["i2c"]),
            bool(fields["spi"]),
        ),
        cast("int", fields["frequency_hz"]),
        tuple(calibrations[label] for label in CALIBRATION_LABELS),
    )


def _quantize(value: float, scale: float, minimum: int, maximum: int, field: str) -> int:
    if not math.isfinite(value):
        raise ValueError(f"{field} is not finite")
    scaled = value * scale
    result = round(scaled)
    if abs(scaled - result) > 0.02:
        raise ValueError(
            f"{field}={value!r} is not an uncalibrated sensor value "
            f"(nearest raw bin differs by {abs(scaled - result):.4g})"
        )
    if not minimum <= result <= maximum:
        raise ValueError(f"{field}={value!r} reconstructs outside [{minimum}, {maximum}]")
    return result


def _pack_barometer(row: dict[str, float], context: str) -> bytes:
    temperature = _quantize(row["temperature"], 65536.0, 0, 0xFFFFFF, f"{context}:temperature")
    pressure = _quantize(row["pressure"], 64.0, 0, 0xFFFFFF, f"{context}:pressure")
    return temperature.to_bytes(3, "little") + pressure.to_bytes(3, "little")


def _pack_20_bit(values: tuple[int, int, int], other: tuple[int, int, int]) -> bytes:
    first = tuple(value & 0xFFFFF for value in values)
    second = tuple(value & 0xFFFFF for value in other)
    payload = bytearray(15)
    for axis, value in enumerate(first):
        payload[axis * 2] = (value >> 12) & 0xFF
        payload[axis * 2 + 1] = (value >> 4) & 0xFF
        payload[12 + axis] = (value & 0x0F) << 4
    for axis, value in enumerate(second):
        payload[6 + axis * 2] = (value >> 12) & 0xFF
        payload[7 + axis * 2] = (value >> 4) & 0xFF
        payload[12 + axis] |= value & 0x0F
    return bytes(payload)


def _pack_imu(row: dict[str, float], context: str) -> bytes:
    accel = (
        _quantize(row["accel_x"], 16384.0, -(1 << 19), (1 << 19) - 1, f"{context}:accel_x"),
        _quantize(row["accel_y"], 16384.0, -(1 << 19), (1 << 19) - 1, f"{context}:accel_y"),
        _quantize(row["accel_z"], 16384.0, -(1 << 19), (1 << 19) - 1, f"{context}:accel_z"),
    )
    gyro = (
        _quantize(row["gyro_x"], 131.072, -(1 << 19), (1 << 19) - 1, f"{context}:gyro_x"),
        _quantize(row["gyro_y"], 131.072, -(1 << 19), (1 << 19) - 1, f"{context}:gyro_y"),
        _quantize(row["gyro_z"], 131.072, -(1 << 19), (1 << 19) - 1, f"{context}:gyro_z"),
    )
    return _pack_20_bit(accel, gyro)


def _pack_magnetometer(row: dict[str, float], context: str) -> bytes:
    values = tuple(
        _quantize(
            row[f"mag_{axis}"] + 131072.0 / 163.84,
            163.84,
            0,
            (1 << 18) - 1,
            f"{context}:mag_{axis}",
        )
        for axis in "xyz"
    )
    payload = bytearray(7)
    for axis, value in enumerate(values):
        payload[axis * 2] = (value >> 10) & 0xFF
        payload[axis * 2 + 1] = (value >> 2) & 0xFF
    payload[6] = ((values[0] & 3) << 6) | ((values[1] & 3) << 4) | (values[2] & 0x0C)
    return bytes(payload)


def _pack_high_g(row: dict[str, float], context: str) -> bytes:
    values = tuple(
        _quantize(row[f"accel_{axis}"], 10.24, -(1 << 11), (1 << 11) - 1, f"{context}:accel_{axis}")
        & 0xFFF
        for axis in "xyz"
    )
    payload = bytearray(6)
    for axis, value in enumerate(values):
        payload[axis * 2] = (value >> 4) & 0xFF
        payload[axis * 2 + 1] = (value & 0x0F) << 4
    return bytes(payload)


SENSORS = (
    SensorSpec("BMP581_data.csv", b"B", ("timestamp", "temperature", "pressure"), _pack_barometer),
    SensorSpec(
        "ICM45686_data.csv",
        b"I",
        ("timestamp", "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"),
        _pack_imu,
    ),
    SensorSpec(
        "MMC5983MA_data.csv",
        b"M",
        ("timestamp", "mag_x", "mag_y", "mag_z"),
        _pack_magnetometer,
    ),
    SensorSpec(
        "ADXL371_data.csv", b"A", ("timestamp", "accel_x", "accel_y", "accel_z"), _pack_high_g
    ),
)


def _events(
    path: Path, spec: SensorSpec, priority: int
) -> Iterator[tuple[float, int, int, bytes, bytes]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.reader(handle)
        _header_line_number = 0
        for _header_line_number, row in enumerate(reader, start=1):
            if row and row[0].strip().lower() == "timestamp":
                break
        else:
            raise ValueError(f"{path}: timestamp column header was not found")

        previous = -math.inf
        sequence = 0
        for line_number, row in enumerate(reader, start=_header_line_number + 1):
            if not row or not any(value.strip() for value in row):
                continue
            if len(row) != len(spec.columns):
                raise ValueError(
                    f"{path}:{line_number}: expected {len(spec.columns)} columns, got {len(row)}"
                )
            try:
                values = {name: float(value) for name, value in zip(spec.columns, row, strict=True)}
            except ValueError as error:
                raise ValueError(f"{path}:{line_number}: non-numeric sensor row") from error
            timestamp = values["timestamp"]
            if not math.isfinite(timestamp) or timestamp < 0:
                raise ValueError(f"{path}:{line_number}: timestamp must be finite and non-negative")
            if timestamp < previous:
                raise ValueError(f"{path}:{line_number}: timestamps are not sorted")
            previous = timestamp
            context = f"{path}:{line_number}"
            yield timestamp, priority, sequence, spec.identifier, spec.pack(values, context)
            sequence += 1


def _write_header(handle, header: Header) -> None:
    handle.write(b"FIRM LOG v1.4\n")
    handle.write(header.uid.to_bytes(8, "little"))
    handle.write(header.device_name.encode("utf-8", errors="replace")[:32].ljust(32, b"\0"))
    handle.write(bytes(int(value) for value in header.communications))
    handle.write(header.firmware_version.encode("ascii", errors="replace")[:8].ljust(8, b"\0"))
    handle.write(struct.pack("<H", header.frequency_hz))
    calibration = tuple(value for sensor in header.calibration for value in sensor)
    handle.write(struct.pack("<48f", *calibration))


def _write_log(
    header: Header,
    streams: Iterable[Iterator[tuple[float, int, int, bytes, bytes]]],
    output: Path,
    *,
    force: bool,
) -> tuple[int, float]:
    output = output.resolve()
    if output.exists() and not force:
        raise FileExistsError(f"output already exists: {output} (pass --force to replace it)")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.{uuid.uuid4().hex}.tmp")
    count = 0
    final_timestamp = 0.0
    try:
        with temporary.open("wb") as handle:
            _write_header(handle, header)
            for timestamp, _priority, _sequence, identifier, payload in heapq.merge(*streams):
                ticks = round(timestamp * CPU_HZ)
                handle.write(identifier)
                handle.write((ticks & 0xFFFFFFFF).to_bytes(4, "little"))
                handle.write(payload)
                count += 1
                final_timestamp = timestamp
                if count % 1_000_000 == 0:
                    print(f"  wrote {count:,} packets ({final_timestamp:.2f} s)", flush=True)
            handle.flush()
            os.fsync(handle.fileno())
        if count == 0:
            raise ValueError("the CSV files contain no sensor rows")
        temporary.replace(output)
    finally:
        temporary.unlink(missing_ok=True)
    return count, final_timestamp


def reconstruct(directory: Path, output: Path, *, force: bool = False) -> tuple[int, float]:
    """Stream the four canonical flight CSVs into an atomically replaced v1.4 log."""
    directory = directory.resolve()
    output = output.resolve()
    paths = [directory / spec.filename for spec in SENSORS]
    missing = [str(path) for path in paths if not path.is_file()]
    if missing:
        raise FileNotFoundError("missing required CSV files:\n  " + "\n  ".join(missing))

    headers = [read_header(path, spec.columns) for path, spec in zip(paths, SENSORS, strict=True)]
    if any(header != headers[0] for header in headers[1:]):
        raise ValueError("the four CSV preambles do not describe the same device and calibration")
    header = headers[0]
    if not 0 <= header.uid <= 0xFFFFFFFFFFFFFFFF:
        raise ValueError("device UID does not fit in the v1.4 header")
    if not 0 <= header.frequency_hz <= 0xFFFF:
        raise ValueError("transmit frequency does not fit in the v1.4 header")

    streams = [
        _events(path, spec, priority)
        for priority, (path, spec) in enumerate(zip(paths, SENSORS, strict=True))
    ]
    return _write_log(header, streams, output, force=force)


def reconstruct_magnetometer_calibration(
    csv_path: Path, output: Path, *, force: bool = False
) -> tuple[int, float]:
    """Reconstruct a v1.4 calibration log containing only magnetometer packets."""
    csv_path = csv_path.resolve()
    if not csv_path.is_file():
        raise FileNotFoundError(f"magnetometer-calibration CSV does not exist: {csv_path}")
    spec = SENSORS[2]
    header = read_header(csv_path, spec.columns)
    if not 0 <= header.uid <= 0xFFFFFFFFFFFFFFFF:
        raise ValueError("device UID does not fit in the v1.4 header")
    if not 0 <= header.frequency_hz <= 0xFFFF:
        raise ValueError("transmit frequency does not fit in the v1.4 header")
    return _write_log(header, [_events(csv_path, spec, 0)], output, force=force)


def main() -> int:
    """Run the disposable CSV-to-FRM recovery command."""
    parser = argparse.ArgumentParser(
        description="Reconstruct a compact FIRM v1.4 .frm file from four legacy decoder CSVs"
    )
    parser.add_argument(
        "csv_directory",
        type=Path,
        nargs="?",
        help=(
            "directory containing BMP581_data.csv, ICM45686_data.csv, "
            "MMC5983MA_data.csv, and ADXL371_data.csv"
        ),
    )
    parser.add_argument("--output", "-o", type=Path, default=Path("reconstructed.frm"))
    parser.add_argument(
        "--mag-cal-csv",
        type=Path,
        help="legacy timestamp/mag_x/mag_y/mag_z CSV for a calibration recording",
    )
    parser.add_argument(
        "--mag-cal-output",
        type=Path,
        default=Path("reconstructed-magnetometer-calibration.frm"),
    )
    parser.add_argument("--force", action="store_true", help="replace an existing output file")
    args = parser.parse_args()
    if args.csv_directory is None and args.mag_cal_csv is None:
        parser.error("provide a flight CSV directory, --mag-cal-csv, or both")
    if (
        args.csv_directory is not None
        and args.mag_cal_csv is not None
        and args.output.resolve() == args.mag_cal_output.resolve()
    ):
        parser.error("--output and --mag-cal-output must be different files")
    try:
        if args.csv_directory is not None:
            count, duration = reconstruct(args.csv_directory, args.output, force=args.force)
            print(
                f"Wrote {args.output.resolve()} "
                f"({count:,} packets, {duration:.2f} s, FIRM log v1.4)"
            )
        if args.mag_cal_csv is not None:
            count, duration = reconstruct_magnetometer_calibration(
                args.mag_cal_csv, args.mag_cal_output, force=args.force
            )
            print(
                f"Wrote {args.mag_cal_output.resolve()} "
                f"({count:,} magnetometer packets, {duration:.2f} s, FIRM log v1.4)"
            )
    except (FileExistsError, FileNotFoundError, OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    print(
        "Keep the source CSVs: this recovered log is reconstructed, not the original acquisition."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
