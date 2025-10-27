"""Parser for FIRM data packets from a serial interface."""

import struct

import serial
import threading
import queue

from .constants import (
    CRC16_TABLE,
    CRC_SIZE,
    FULL_PACKET_SIZE,
    HEADER_SIZE,
    LENGTH_FIELD_SIZE,
    PADDING_SIZE,
    PAYLOAD_LENGTH,
    START_BYTE,
)
from .packets import BarometerPacket, IMUPacket, MagnetometerPacket


class PacketParser:
    """Parser for FIRM data packets from a serial interface.

    TODO: Write more docs here.

    """

    __slots__ = ("_bytes_stored", "_ser", "_struct", "_rx_thread", "_stop_event", "_pkt_q")

    def __init__(self, port: str, baudrate: int):
        self._ser = serial.Serial(port, baudrate)
        self._bytes_stored = bytearray()
        self._struct = struct.Struct("<fffffffffffxxxxd")
        self._rx_thread = None
        self._stop_event = threading.Event()
        self._pkt_q: queue.Queue[IMUPacket | BarometerPacket | MagnetometerPacket] = queue.Queue(maxsize=8192)

    def __enter__(self):
        """Context manager entry: initialize the parser."""
        self.initialize()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Context manager exit: Ensure serial port is closed on exit."""
        self.close()

    def initialize(self):
        """Open serial and prepare for parsing packets."""
        if not self._ser.is_open:
            self._ser.open()
        self._bytes_stored.clear()
        self._stop_event.clear()
        if self._rx_thread is None or not self._rx_thread.is_alive():
            self._rx_thread = threading.Thread(target=self._worker, name="FIRM-RX", daemon=True)
            self._rx_thread.start()

    def close(self):
        """Close the serial port."""
        self._stop_event.set()
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None
        if self._ser.is_open:
            self._ser.close()

    def get_data_packets(
            self, block: bool = True, timeout: float | None = None, max_items: int | None = None
    ) -> list[IMUPacket | BarometerPacket | MagnetometerPacket]:
        """
        Retrieve packets parsed by the background thread.

        Args:
            block: If True, wait for at least one packet.
            timeout: Seconds to wait for the first packet if block=True.
            max_items: Optional cap on number of packets returned.

        Returns:
            List of parsed packets.
        """
        out: list[IMUPacket | BarometerPacket | MagnetometerPacket] = []

        if block:
            try:
                first = self._pkt_q.get(timeout=timeout)
                out.append(first)
            except queue.Empty:
                return out

        try:
            while max_items is None or len(out) < max_items:
                out.append(self._pkt_q.get_nowait())
        except queue.Empty:
            pass

        return out

    def _parse_packets(self) -> list[IMUPacket | BarometerPacket | MagnetometerPacket]:
        """Attempt to parse packets from the data. If any packets cannot be fully parsed, the data
        is retained for the next read."""

        packets = []
        pos = 0
        data_len = len(self._bytes_stored)
        view = memoryview(self._bytes_stored)

        while pos < data_len:
            # Find the next header starting from pos
            header_pos = self._bytes_stored.find(START_BYTE, pos)
            if header_pos == -1:  # No more headers found (incomplete packet)
                break

            # Check if we have enough data for a complete packet
            if header_pos + FULL_PACKET_SIZE > data_len:
                break

            # Parse and validate length of payload
            length_start = header_pos + HEADER_SIZE
            length = int.from_bytes(view[length_start : length_start + LENGTH_FIELD_SIZE], "little")

            if length != PAYLOAD_LENGTH:
                pos = length_start
                continue

            # Calculate packet boundaries
            payload_start = length_start + LENGTH_FIELD_SIZE + PADDING_SIZE
            crc_start = payload_start + length

            # Verify CRC
            if not self._verify_crc(view, header_pos, crc_start):
                pos = length_start
                continue

            # Extract and parse payload
            payload = bytes(view[payload_start:crc_start])
            packet_group = self._create_packet_group(payload)
            if packet_group:
                packets.extend(packet_group)

            pos = crc_start + CRC_SIZE

        # Retain unparsed data, delete parsed bytes:
        self._bytes_stored = self._bytes_stored[pos:]

        return packets

    def _create_packet_group(
        self, payload: bytes
    ) -> list[IMUPacket | BarometerPacket | MagnetometerPacket] | None:
        """Unpack payload and create packet group."""
        try:
            fields = self._struct.unpack(payload)
        except (struct.error, ValueError):
            return None

        (temp, press, ax, ay, az, gx, gy, gz, mx, my, mz, ts) = fields

        return [
            IMUPacket(ts, ax, ay, az, gx, gy, gz),
            BarometerPacket(ts, press, temp),
            MagnetometerPacket(ts, mx, my, mz),
        ]

    def _verify_crc(self, data: memoryview, header_pos: int, crc_start: int) -> bool:
        """Verify CRC checksum for packet."""
        data_for_crc = data[header_pos:crc_start]
        received_crc = int.from_bytes(data[crc_start : crc_start + CRC_SIZE], "little")
        return self._crc16_ccitt(data_for_crc) == received_crc

    def _crc16_ccitt(self, data: bytes) -> int:
        """Compute the CRC-16-CCITT checksum for the given data."""
        crc = 0x0000
        for byte in data:
            idx = (crc ^ byte) & 0xFF
            crc = (CRC16_TABLE[idx] ^ (crc >> 8)) & 0xFFFF
        return crc
