"""A Python stand-in for the Teensy firmware.

It speaks the protocol in docs/protocol.md and acts like a Teensy with perfect timing: each sample
plays exactly when it is due, unless it arrived late. It is used like a serial port, so the laptop
code cannot tell it apart from the real board. Nothing runs in the background; it catches up on
playback whenever it is read from or written to.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING

from firm_emu.log import Record
from firm_emu.protocol import (
    DONE,
    END,
    ERR_BAD_FRAME,
    ERR_BAD_SAMPLE,
    ERR_BUFFER_FULL,
    ERR_UNKNOWN_TYPE,
    ERROR,
    EVENT,
    GET_STATUS,
    IDLE,
    LOAD,
    PING,
    PONG,
    RUNNING,
    START,
    STATUS,
    STOP,
    VERSION,
    Event,
    FrameError,
    FrameReader,
    Status,
    decode_frame,
    encode_frame,
    unpack_samples,
)

if TYPE_CHECKING:
    from firm_emu.replay import Clock

BUFFER_BYTES = 256 * 1024
SAMPLE_HEADER = 5  # sensor ID + uint32 due time


@dataclass(frozen=True)
class Buffered:
    """A sample waiting in the buffer."""

    record: Record
    arrived_s: float

    @property
    def size(self) -> int:
        """Bytes it takes in the buffer."""
        return SAMPLE_HEADER + len(self.record.payload)


class FakeTeensy:
    """Stand-in for the Teensy. `played` holds every sample with the time it was played."""

    def __init__(self, clock: Clock, buffer_bytes: int = BUFFER_BYTES):
        self.clock = clock
        self.capacity = buffer_bytes
        self._reader = FrameReader()
        self._outbox = bytearray()
        self._reset()

    def _reset(self) -> None:
        """Back to idle with an empty buffer, as after power-up or STOP."""
        self.buffer: deque[Buffered] = deque()
        self.used = 0
        self.state = IDLE
        self.ended = False
        self.start_s = 0.0
        self.played: list[Record] = []
        self.late = 0
        self.max_late_s = 0.0

    # --- serial port interface ---

    def write(self, data: bytes) -> int:
        """Receive bytes from the laptop."""
        self._catch_up()
        for frame in self._reader.feed(data):
            self._handle(frame)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        """Return bytes waiting for the laptop."""
        self._catch_up()
        data = bytes(self._outbox[:size])
        del self._outbox[:size]
        return data

    # --- what the firmware does ---

    def emit_event(self, kind: int, value: int) -> None:
        """Report a FIRM output change, as the real Teensy does when it sees one."""
        self._send(EVENT, Event(kind, self._time_us(), value).pack())

    def _handle(self, frame: bytes) -> None:
        try:
            kind, payload = decode_frame(frame)
        except FrameError:
            self._send(ERROR, bytes([ERR_BAD_FRAME]))
            return
        if kind == PING:
            self._send(PONG, bytes([VERSION]))
        elif kind == LOAD:
            self._load(payload)
        elif kind == START:
            self.state, self.start_s = RUNNING, self.clock.now()
        elif kind == END:
            self.ended = True
        elif kind == STOP:
            self._reset()
        elif kind == GET_STATUS:
            self._send(STATUS, self._status().pack())
        else:
            self._send(ERROR, bytes([ERR_UNKNOWN_TYPE]))

    def _load(self, payload: bytes) -> None:
        if len(payload) > self.capacity - self.used:
            self._send(ERROR, bytes([ERR_BUFFER_FULL]))
            return
        try:
            records = unpack_samples(payload)
        except ValueError:
            self._send(ERROR, bytes([ERR_BAD_SAMPLE]))
            return
        now = self.clock.now()
        for record in records:
            sample = Buffered(record, now)
            self.buffer.append(sample)
            self.used += sample.size

    def _catch_up(self) -> None:
        """Play every sample that is due by now."""
        if self.state != RUNNING:
            return
        now = self.clock.now()
        while self.buffer and self.start_s + self.buffer[0].record.time_s <= now:
            sample = self.buffer.popleft()
            self.used -= sample.size
            due = self.start_s + sample.record.time_s
            if sample.arrived_s > due:
                self.late += 1
                self.max_late_s = max(self.max_late_s, sample.arrived_s - due)
            played = max(due, sample.arrived_s) - self.start_s
            self.played.append(Record(sample.record.sensor, played, sample.record.payload))
        if self.ended and not self.buffer:
            self.state = DONE

    def _status(self) -> Status:
        return Status(
            self.state,
            self.capacity - self.used,
            len(self.played),
            self.late,
            round(self.max_late_s * 1e6),
            self._time_us(),
        )

    def _time_us(self) -> int:
        return round((self.clock.now() - self.start_s) * 1e6) if self.state != IDLE else 0

    def _send(self, kind: int, payload: bytes) -> None:
        self._outbox += encode_frame(kind, payload)
