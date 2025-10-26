"""Parser for FIRM data packets from a serial interface."""

import struct

import serial

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

    __slots__ = ("_bytes_stored", "_ser", "_struct")

    def __init__(self, port: str, baudrate: int):
        self._ser = serial.Serial(port, baudrate)
        self._bytes_stored = bytearray()
        self._struct = struct.Struct("<fffffffffffxxxxd")

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

    def close(self):
        """Close the serial port."""
        if self._ser.is_open:
            self._ser.close()

    def get_data_packets(
        self, block: bool = True
    ) -> list[IMUPacket | BarometerPacket | MagnetometerPacket]:
        """Read data from the serial port and parse packets.

        Args:
            block (bool): If True, block until at least one packet is read.

        Returns:
            list: A list of parsed msgspec packets. Currently, the packets can be any of
                IMUPacket, BarometerPacket, and MagnetometerPacket.
        """
        packets = []

        if block:
            while not packets:
                chunk = self._ser.read(self._ser.in_waiting)
                # Add the read bytes to the stored bytes
                self._bytes_stored.extend(chunk)
                # Attempt to parse packets from the stored bytes:
                packets = self._parse_packets()
                if packets:
                    break
        elif self._ser.in_waiting > 0:
            chunk = self._ser.read(self._ser.in_waiting)
            self._bytes_stored.extend(chunk)
            packets = self._parse_packets()

        return packets

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
            BarometerPacket(ts, temp, press),
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
