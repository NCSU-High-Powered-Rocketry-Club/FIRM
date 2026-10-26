"""Replay log records at their original timing.

The same scheduling will run on the Teensy. Here it lets the timing be tested on a laptop: the
file output records when each record was actually sent, so `firm-emu compare` can check it.
"""

from __future__ import annotations

import gc
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Protocol

from firm_emu.log import Record, write_log


class Clock(Protocol):
    """Where replay gets the time from. Tests use a fake one."""

    def now(self) -> float:
        """Current time in seconds."""

    def wait_until(self, deadline: float) -> None:
        """Return once now() has reached the deadline."""


class RealClock:
    """Sleeps for most of the wait, then spins for the last few milliseconds to be precise.

    time.sleep on Windows sometimes wakes up more than 1 ms late, so the spin covers 3 ms.
    """

    SPIN_S = 0.003

    def now(self) -> float:
        """Current time in seconds."""
        return time.perf_counter()

    def wait_until(self, deadline: float) -> None:
        """Return as close as possible to the deadline."""
        while (remaining := deadline - time.perf_counter()) > 0:
            if remaining > self.SPIN_S + 0.001:
                time.sleep(remaining - self.SPIN_S)


class Output(Protocol):
    """Where replayed records go: a file now, the Teensy later."""

    def send(self, record: Record) -> None:
        """Send one record. Its time_s is when it was actually sent."""

    def close(self) -> None:
        """Called once after the last record."""


class FileOutput:
    """Collects sent records, stamped with the time they were sent, and writes a .frm log."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.records: list[Record] = []

    def send(self, record: Record) -> None:
        """Keep the record for the output log."""
        self.records.append(record)

    def close(self) -> None:
        """Write the output log."""
        write_log(self.path, self.records, device_name="replay")


@dataclass(frozen=True)
class ReplayStats:
    """How well the replay kept time."""

    sent: int
    duration_s: float  # real seconds the replay took
    max_late_ms: float  # worst delay past a record's due time, in real time


def replay(
    records: list[Record], output: Output, speed: float = 1.0, clock: Clock | None = None
) -> ReplayStats:
    """Send each record when it is due. speed=2 replays twice as fast.

    Each record is sent with its time replaced by the time it was actually sent (in log seconds),
    so the output shows exactly how the replay kept time.
    """
    clock = clock or RealClock()
    gc_was_enabled = gc.isenabled()
    gc.disable()  # a garbage collection pass over a big log can pause for tens of ms
    try:
        start = clock.now()
        max_late = 0.0
        for record in records:
            due = start + record.time_s / speed
            clock.wait_until(due)
            sent = clock.now()
            max_late = max(max_late, sent - due)
            output.send(replace(record, time_s=(sent - start) * speed))
        duration = clock.now() - start
    finally:
        if gc_was_enabled:
            gc.enable()
    output.close()
    return ReplayStats(len(records), duration, max_late * 1000)
