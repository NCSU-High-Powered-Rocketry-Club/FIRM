"""Parser for FIRM data packets from a serial interface."""

import struct
from typing import Literal, overload

import serial

from .constants import CRC16_TABLE, START_BYTE
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
        self.initialize()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Ensure serial port is closed on exit."""
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

    @overload
    def get_data_packets(
        self, block: Literal[True] = True
    ) -> list[IMUPacket | BarometerPacket | MagnetometerPacket]: ...
    @overload
    def get_data_packets(
        self, block: Literal[False] = False
    ) -> list[IMUPacket | BarometerPacket | MagnetometerPacket] | None: ...

    def get_data_packets(
        self, block: bool = True
    ) -> list[IMUPacket | BarometerPacket | MagnetometerPacket] | None:
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

    def _parse_packets(self) -> list[IMUPacket | BarometerPacket | MagnetometerPacket] | None:
        """Attempt to parse packets from the data. If any packets cannot be fully parsed, the data
        is retained for the next read."""

        packets = []
        pos = 0
        data_len = len(self._bytes_stored)
        while pos < data_len:
            # Find the next header starting from pos
            header_pos = self._bytes_stored.find(START_BYTE, pos)
            if header_pos == -1:  # No more headers found (incomplete packet)
                break

            pos = header_pos + 2
            if pos + 2 > data_len:  # Not enough data for length field
                break

            length_bytes = self._bytes_stored[pos : pos + 2]
            length = int.from_bytes(length_bytes, "little")
            if length != 56:
                pos += 2  # Skip length field and continue searching
                continue

            # Check if full packet is available:
            payload_start = pos + 6
            payload_end = payload_start + length
            crc_start = payload_end
            if crc_start + 2 > data_len:
                break

            # Verify CRC
            data_for_crc = self._bytes_stored[header_pos:crc_start]
            received_crc = int.from_bytes(self._bytes_stored[crc_start : crc_start + 2], "little")
            computed_crc = self._crc16_ccitt(data_for_crc)
            if computed_crc != received_crc:
                pos = header_pos + 2
                continue

            # Extract payload
            payload = self._bytes_stored[payload_start:payload_end]
            try:
                fields = self._struct.unpack(payload)
                (
                    temperature,
                    pressure,
                    accel_x,
                    accel_y,
                    accel_z,
                    angular_rate_x,
                    angular_rate_y,
                    angular_rate_z,
                    magnetic_field_x,
                    magnetic_field_y,
                    magnetic_field_z,
                    timestamp,
                ) = fields
                imu_packet = IMUPacket(
                    timestamp_secs=timestamp,
                    acc_x_gs=accel_x,
                    acc_y_gs=accel_y,
                    acc_z_gs=accel_z,
                    gyro_x_rad_s=angular_rate_x,
                    gyro_y_rad_s=angular_rate_y,
                    gyro_z_rad_s=angular_rate_z,
                )
                baro_packet = BarometerPacket(
                    timestamp_secs=timestamp,
                    temperature_celsius=temperature,
                    pressure_pascals=pressure,
                )
                mag_packet = MagnetometerPacket(
                    timestamp_secs=timestamp,
                    mag_x_ut=magnetic_field_x,
                    mag_y_ut=magnetic_field_y,
                    mag_z_ut=magnetic_field_z,
                )
                packets.extend([imu_packet, baro_packet, mag_packet])
                # Advance position past this packet
                pos = crc_start + 2
            except (struct.error, ValueError) as e:
                # Unpacking failed, skip and continue searching after length
                pos = header_pos + 2
                raise (e)
                continue

        # Retain unparsed data, delete parsed bytes:
        self._bytes_stored = self._bytes_stored[pos:]

        return packets if packets else None

    def _crc16_ccitt(self, data: bytes) -> int:
        """Compute the CRC-16-CCITT checksum for the given data."""
        crc = 0x0000
        for byte in data:
            idx = (crc ^ byte) & 0xFF
            crc = (CRC16_TABLE[idx] ^ (crc >> 8)) & 0xFFFF
        return crc
