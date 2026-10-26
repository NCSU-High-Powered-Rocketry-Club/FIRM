"""Laptop side of the laptop - Teensy protocol. See docs/protocol.md."""

from __future__ import annotations

import struct
from binascii import crc_hqx
from dataclasses import dataclass
from typing import TYPE_CHECKING, Protocol

from firm.flight_data.formats import PACKET_SIZES

from firm_emu.log import SENSOR_IDS, Record

if TYPE_CHECKING:
    from firm_emu.replay import Clock

VERSION = 1
MAX_PAYLOAD = 1024

# laptop to Teensy
PING, LOAD, START, END, STOP, GET_STATUS = 0x01, 0x02, 0x03, 0x04, 0x05, 0x06
# Teensy to laptop
PONG, STATUS, EVENT, ERROR = 0x81, 0x82, 0x83, 0x84

IDLE, RUNNING, DONE = 0, 1, 2
ERR_BAD_FRAME, ERR_UNKNOWN_TYPE, ERR_BUFFER_FULL, ERR_BAD_SAMPLE = 1, 2, 3, 4
LED_EVENT = 1

STATUS_FORMAT = "<BIIIII"
EVENT_FORMAT = "<BIB"


class FrameError(ValueError):
    """A frame failed COBS decoding or its CRC check."""


class TeensyError(RuntimeError):
    """The Teensy replied with an ERROR message."""


# --- framing -------------------------------------------------------------------------------


def cobs_encode(data: bytes) -> bytes:
    """Replace every 0x00 so the result can be ended with a single 0x00."""
    out = bytearray([0])
    code_at, code = 0, 1
    for byte in data:
        if byte:
            out.append(byte)
            code += 1
        if not byte or code == 0xFF:
            out[code_at] = code
            code_at, code = len(out), 1
            out.append(0)
    out[code_at] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """Undo cobs_encode."""
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        if code == 0 or i + code > len(data):
            raise FrameError("bad COBS data")
        out += data[i + 1 : i + code]
        i += code
        if code < 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE."""
    return crc_hqx(data, 0xFFFF)


def encode_frame(kind: int, payload: bytes = b"") -> bytes:
    """Build one frame, ready to write to the port."""
    body = bytes([kind]) + payload
    return cobs_encode(body + struct.pack("<H", crc16(body))) + b"\0"


def decode_frame(frame: bytes) -> tuple[int, bytes]:
    """Return (type, payload) of one frame, without its trailing 0x00."""
    data = cobs_decode(frame)
    if len(data) < 3 or struct.unpack("<H", data[-2:])[0] != crc16(data[:-2]):
        raise FrameError("bad CRC")
    return data[0], data[1:-2]


class FrameReader:
    """Collects bytes from the port and hands back complete frames."""

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        """Add received bytes and return every frame that is now complete."""
        self._buffer += data
        *frames, rest = self._buffer.split(b"\0")
        self._buffer = bytearray(rest)
        return [bytes(f) for f in frames if f]


# --- messages ------------------------------------------------------------------------------


def pack_sample(record: Record) -> bytes:
    """One record as a LOAD sample."""
    time_us = round(record.time_s * 1e6)
    return bytes([SENSOR_IDS[record.sensor]]) + struct.pack("<I", time_us) + record.payload


def unpack_samples(payload: bytes) -> list[Record]:
    """Split a LOAD payload back into records."""
    records = []
    i = 0
    while i < len(payload):
        if payload[i] not in PACKET_SIZES:
            raise ValueError(f"unknown sensor ID {payload[i]}")
        sensor, size = PACKET_SIZES[payload[i]]
        if i + 5 + size > len(payload):
            raise ValueError("sample cut short")
        (time_us,) = struct.unpack_from("<I", payload, i + 1)
        records.append(Record(sensor, time_us / 1e6, payload[i + 5 : i + 5 + size]))
        i += 5 + size
    return records


@dataclass(frozen=True)
class Status:
    """The Teensy's STATUS reply."""

    state: int
    free_bytes: int
    played: int
    late: int
    max_late_us: int
    time_us: int

    def pack(self) -> bytes:
        """STATUS payload."""
        return struct.pack(STATUS_FORMAT, *vars(self).values())


@dataclass(frozen=True)
class Event:
    """A FIRM output the Teensy saw change, such as the debug LEDs."""

    kind: int
    time_us: int
    value: int

    def pack(self) -> bytes:
        """EVENT payload."""
        return struct.pack(EVENT_FORMAT, self.kind, self.time_us, self.value)


# --- laptop client -------------------------------------------------------------------------


class Port(Protocol):
    """A pyserial Serial opened with timeout=0, or a FakeTeensy."""

    def write(self, data: bytes) -> int | None:
        """Send bytes."""

    def read(self, size: int = 1) -> bytes:
        """Return up to size bytes that have arrived, without waiting."""


class Teensy:
    """Talks to the Teensy (or the fake one) over a port."""

    def __init__(self, port: Port, clock: Clock, timeout_s: float = 1.0):
        self.port = port
        self.clock = clock
        self.timeout_s = timeout_s
        self.reader = FrameReader()
        self.events: list[Event] = []

    def send(self, kind: int, payload: bytes = b"") -> None:
        """Send one message."""
        self.port.write(encode_frame(kind, payload))

    def receive(self, kind: int) -> bytes:
        """Wait for a message of this type. EVENTs are kept in self.events, ERRORs raise."""
        deadline = self.clock.now() + self.timeout_s
        while self.clock.now() < deadline:
            found = None
            for frame in self.reader.feed(self.port.read(4096)):
                got, payload = decode_frame(frame)
                if got == EVENT:
                    self.events.append(Event(*struct.unpack(EVENT_FORMAT, payload)))
                elif got == ERROR:
                    raise TeensyError(f"Teensy error code {payload[0]}")
                elif got == kind and found is None:
                    found = payload
            if found is not None:
                return found
            self.clock.wait_until(self.clock.now() + 0.001)
        raise TimeoutError(f"no reply of type {kind:#04x} from the Teensy")

    def ping(self) -> int:
        """Return the Teensy's protocol version."""
        self.send(PING)
        return self.receive(PONG)[0]

    def status(self) -> Status:
        """Ask for the Teensy's state."""
        self.send(GET_STATUS)
        return Status(*struct.unpack(STATUS_FORMAT, self.receive(STATUS)))

    def play(self, records: list[Record], poll_s: float = 0.05) -> Status:
        """Stream records to the Teensy, keeping its buffer topped up, until all have played."""
        samples = [pack_sample(r) for r in records]
        self.send(STOP)  # clear anything left from an earlier run
        sent = self._fill(samples, 0)
        self.send(START)
        while sent < len(samples):
            self.clock.wait_until(self.clock.now() + poll_s)
            sent = self._fill(samples, sent)
        self.send(END)
        while (status := self.status()).state != DONE:
            self.clock.wait_until(self.clock.now() + poll_s)
        return status

    def _fill(self, samples: list[bytes], sent: int) -> int:
        """Send as many samples as fit in the buffer. Returns how many have been sent in total."""
        free = self.status().free_bytes
        while sent < len(samples):
            frame = bytearray()
            while sent < len(samples) and len(frame) + len(samples[sent]) <= min(MAX_PAYLOAD, free):
                frame += samples[sent]
                sent += 1
            if not frame:
                break
            self.send(LOAD, bytes(frame))
            free -= len(frame)
        return sent
