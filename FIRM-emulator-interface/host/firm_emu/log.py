"""Read, write, and compare FIRM .frm logs.

Reading uses the team's decoder in firm.flight_data, so every log version it supports works here.
See docs/log-format.md for the file layout.
"""

from __future__ import annotations

import statistics
import struct
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path
from typing import TYPE_CHECKING

from firm.flight_data.formats import CPU_HZ, PACKET_SIZES, LogReader

if TYPE_CHECKING:
    from firm.flight_data.models import LogHeader

SENSORS = ("barometer", "imu", "magnetometer", "high_g")
SENSOR_IDS = {name: identifier for identifier, (name, _size) in PACKET_SIZES.items()}
BLOCK_SIZE = 8192
MAX_MISSING_FRACTION = 0.01


@dataclass(frozen=True, slots=True)
class Record:
    """One logged sensor read."""

    sensor: str
    time_s: float  # seconds since the first record in the log
    payload: bytes  # raw register bytes


def read_log(path: str | Path) -> tuple[LogHeader, list[Record]]:
    """Return the header and every record up to the end of data."""
    reader = LogReader(path)
    records = [Record(p.sensor, p.timestamp_s, p.payload) for p in reader.iter_packets()]
    return reader.header, records


def write_log(path: str | Path, records: list[Record], device_name: str = "emulator") -> None:
    """Write records as a v1.4 log, laid out in 8192-byte blocks the way FIRM does."""
    identity = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    out = bytearray(
        b"FIRM LOG v1.4\n"
        + bytes(8)  # device UID
        + device_name.encode().ljust(32, b"\0")
        + bytes(4)  # USB, UART, I2C, SPI output enabled
        + b"v0.0.0".ljust(8, b"\0")
        + struct.pack("<H", 100)  # output frequency
        + struct.pack("<48f", *identity * 4)  # accel, gyro, mag, high-g calibration
    )
    for record in records:
        ticks = round(record.time_s * CPU_HZ) % 2**32
        entry = bytes([SENSOR_IDS[record.sensor]]) + struct.pack("<I", ticks) + record.payload
        room = BLOCK_SIZE - len(out) % BLOCK_SIZE
        if len(entry) > room:  # records never cross a block, the rest of the block is zeros
            out += bytes(room)
        out += entry
    out += bytes(-len(out) % BLOCK_SIZE)
    Path(path).write_bytes(out)


def by_sensor(records: list[Record]) -> dict[str, list[Record]]:
    """Split records into one time-ordered list per sensor."""
    groups: dict[str, list[Record]] = {name: [] for name in SENSORS}
    for record in records:
        groups[record.sensor].append(record)
    return groups


@dataclass(frozen=True)
class SensorStats:
    """Sample count and timing for one sensor."""

    count: int
    median_interval_ms: float
    max_gap_ms: float

    @property
    def rate_hz(self) -> float:
        """Sample rate implied by the median interval."""
        return 1000 / self.median_interval_ms if self.median_interval_ms else 0.0


def sensor_stats(records: list[Record]) -> SensorStats:
    """Summarize one sensor's records."""
    intervals = [(b.time_s - a.time_s) * 1000 for a, b in pairwise(records)]
    if not intervals:
        return SensorStats(len(records), 0.0, 0.0)
    return SensorStats(len(records), statistics.median(intervals), max(intervals))


@dataclass
class SensorDiff:
    """How one sensor's records in the actual log line up with the expected log."""

    matched: int = 0  # same bytes, on time
    late: int = 0  # same bytes, but further off in time than the tolerance
    missing: int = 0  # expected, but not in the actual log
    extra: int = 0  # in the actual log, but not expected
    wrong: int = 0  # at the expected time, but with different bytes
    worst_ms: float = 0.0  # largest timing error of any paired record
    drift_ms: float = 0.0  # how far the two clocks drifted apart by the end
    first_problem_s: float | None = None

    @property
    def passed(self) -> bool:
        """A few missing records are allowed: FIRM drops samples when the SD card is busy."""
        expected = self.matched + self.late + self.missing + self.wrong
        return (
            self.late == 0
            and self.extra == 0
            and self.wrong == 0
            and self.missing <= MAX_MISSING_FRACTION * expected
        )

    def problem_at(self, time_s: float) -> None:
        """Remember the time of the first late, extra, or wrong record."""
        if self.first_problem_s is None:
            self.first_problem_s = time_s


PAIR_WINDOW_S = 0.1  # records with the same bytes this close in time are the same sample
DRIFT_RATE = 0.01  # how quickly the clock offset follows the paired records


def compare_sensor(
    expected: list[Record], actual: list[Record], tolerance_ms: float = 1.0
) -> SensorDiff:
    """Pair records by their bytes, walking both lists in time order, then check the timing.

    The clock offset slowly follows the paired records, so drift between the boards passes but
    a single late record does not shift the reference for the ones after it.
    """
    diff = SensorDiff()
    tolerance = tolerance_ms / 1000
    offset = 0.0

    def pairs(e: Record, a: Record) -> bool:
        return e.payload == a.payload and abs(a.time_s - e.time_s - offset) <= PAIR_WINDOW_S

    i = j = 0
    while i < len(expected) and j < len(actual):
        error = actual[j].time_s - expected[i].time_s - offset
        if pairs(expected[i], actual[j]):
            if abs(error) <= tolerance:
                diff.matched += 1
            else:
                diff.late += 1
                diff.problem_at(expected[i].time_s)
            diff.worst_ms = max(diff.worst_ms, abs(error) * 1000)
            offset += error * DRIFT_RATE
            i += 1
            j += 1
        elif (i + 1 < len(expected) and pairs(expected[i + 1], actual[j])) or error > tolerance:
            diff.missing += 1  # the actual log skipped this record
            i += 1
        elif (j + 1 < len(actual) and pairs(expected[i], actual[j + 1])) or error < -tolerance:
            diff.extra += 1  # the actual log has a record nothing expected
            diff.problem_at(actual[j].time_s)
            j += 1
        else:
            diff.wrong += 1  # right time, different bytes
            diff.problem_at(expected[i].time_s)
            i += 1
            j += 1
    diff.missing += len(expected) - i
    diff.extra += len(actual) - j
    if j < len(actual):
        diff.problem_at(actual[j].time_s)
    diff.drift_ms = offset * 1000
    return diff


def first_seconds(records: list[Record], seconds: float | None) -> list[Record]:
    """Records in the first `seconds` of a log, or all of them if seconds is None."""
    if seconds is None:
        return records
    return [r for r in records if r.time_s < seconds]


def compare_logs(
    expected: str | Path,
    actual: str | Path,
    tolerance_ms: float = 1.0,
    seconds: float | None = None,
) -> dict[str, SensorDiff]:
    """Compare every sensor in two logs, optionally only the first `seconds`. Ignores headers."""
    _, expected_records = read_log(expected)
    _, actual_records = read_log(actual)
    expected_groups = by_sensor(first_seconds(expected_records, seconds))
    actual_groups = by_sensor(first_seconds(actual_records, seconds))
    return {
        name: compare_sensor(expected_groups[name], actual_groups[name], tolerance_ms)
        for name in SENSORS
    }
