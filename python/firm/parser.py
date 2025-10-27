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
from .packets import FIRMPacket


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
        self._pkt_q: queue.Queue[FIRMPacket] = queue.Queue(maxsize=8192)

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
            self,
            block: bool = True,
            timeout: float | None = None,
            max_items: int | None = None,
    ) -> list[FIRMPacket]:
        """
        Retrieve FIRMPacket objects parsed by the background thread.

        Args:
            block: If True, wait for at least one packet.
            timeout: Seconds to wait for the first packet if block=True.
            max_items: Optional cap on number of packets returned.

        Returns:
            List of FIRMPacket objects.
        """
        firm_packets: list[FIRMPacket] = []

        if block:
            try:
                first_packet: FIRMPacket = self._pkt_q.get(timeout=timeout)
                firm_packets.append(first_packet)
            except queue.Empty:
                return firm_packets

        try:
            while max_items is None or len(firm_packets) < max_items:
                firm_packets.append(self._pkt_q.get_nowait())
        except queue.Empty:
            pass

        return firm_packets

    def _parse_packets(self) -> list[FIRMPacket]:
        """Parse as many complete packets as possible and return FIRMPacket objects.
        Any leftover bytes for an incomplete packet are retained for the next read.
        """
        firm_packets: list[FIRMPacket] = []
        position = 0
        data_length = len(self._bytes_stored)
        view = memoryview(self._bytes_stored)

        while position < data_length:
            # Find the next header starting from position
            header_pos = self._bytes_stored.find(START_BYTE, position)
            if header_pos == -1:
                break

            # Not enough bytes for a full packet yet
            if header_pos + FULL_PACKET_SIZE > data_length:
                break

            # Parse and validate the payload length
            length_start = header_pos + HEADER_SIZE
            payload_length = int.from_bytes(
                view[length_start : length_start + LENGTH_FIELD_SIZE], "little"
            )
            if payload_length != PAYLOAD_LENGTH:
                position = length_start
                continue

            # Compute boundaries
            payload_start = length_start + LENGTH_FIELD_SIZE + PADDING_SIZE
            crc_start = payload_start + payload_length

            # CRC check
            if not self._verify_crc(view, header_pos, crc_start):
                position = length_start
                continue

            # Extract payload and build a FIRMPacket
            payload = bytes(view[payload_start:crc_start])
            firm_packet = self._create_firm_packet(payload)  # returns FIRMPacket | None
            if firm_packet is not None:
                firm_packets.append(firm_packet)

            position = crc_start + CRC_SIZE

        # Retain unparsed remainder
        if position:
            del self._bytes_stored[:position]

        return firm_packets

    def _create_firm_packet(self, payload: bytes) -> FIRMPacket | None:
        """Unpack payload and create a single unified FIRMPacket."""
        try:
            fields = self._struct.unpack(payload)
        except (struct.error, ValueError):
            return None

        (
            temperature_celsius,
            pressure_pascals,
            accel_x_meters_per_s2,
            accel_y_meters_per_s2,
            accel_z_meters_per_s2,
            gyro_x_radians_per_s,
            gyro_y_radians_per_s,
            gyro_z_radians_per_s,
            mag_x_microteslas,
            mag_y_microteslas,
            mag_z_microteslas,
            timestamp_seconds,
        ) = fields

        return FIRMPacket(
            timestamp_seconds=timestamp_seconds,
            accel_x_meters_per_s2=accel_x_meters_per_s2,
            accel_y_meters_per_s2=accel_y_meters_per_s2,
            accel_z_meters_per_s2=accel_z_meters_per_s2,
            gyro_x_radians_per_s=gyro_x_radians_per_s,
            gyro_y_radians_per_s=gyro_y_radians_per_s,
            gyro_z_radians_per_s=gyro_z_radians_per_s,
            pressure_pascals=pressure_pascals,
            temperature_celsius=temperature_celsius,
            mag_x_microteslas=mag_x_microteslas,
            mag_y_microteslas=mag_y_microteslas,
            mag_z_microteslas=mag_z_microteslas,
        )

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
