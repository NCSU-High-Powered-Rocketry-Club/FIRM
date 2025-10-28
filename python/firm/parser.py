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


class FIRM:
    """Parser for FIRM data packets from a serial interface.

    TODO: Write more docs here.

    """

    __slots__ = (
        "_bytes_stored",
        "_serial_port",
        "_struct",
        "_serial_reader_thread",
        "_stop_event",
        "_packet_queue",
    )

    def __init__(self, port: str, baudrate: int):
        self._serial_port = serial.Serial(port, baudrate)
        self._bytes_stored = bytearray()
        self._struct = struct.Struct("<fffffffffffxxxxd")
        self._serial_reader_thread = None
        self._stop_event = threading.Event()
        self._packet_queue: queue.Queue[FIRMPacket] = queue.Queue(maxsize=8192)

    def __enter__(self):
        """Context manager entry: initialize the parser."""
        self.initialize()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Context manager exit: Ensure serial port is closed on exit."""
        self.close()

    def initialize(self):
        """Open serial and prepare for parsing packets."""
        if not self._serial_port.is_open:
            self._serial_port.open()
        self._bytes_stored.clear()
        self._stop_event.clear()
        if self._serial_reader_thread is None or not self._serial_reader_thread.is_alive():
            self._serial_reader_thread = threading.Thread(
                target=self._serial_reader, name="FIRM-RX", daemon=True
            )
            self._serial_reader_thread.start()

    def close(self):
        """Close the serial port."""
        self._stop_event.set()
        if self._serial_reader_thread is not None:
            self._serial_reader_thread.join(timeout=1.0)
            self._serial_reader_thread = None
        if self._serial_port.is_open:
            self._serial_port.close()

    def get_data_packets(
        self,
        block: bool = True,
        max_items: int | None = None,
    ) -> list[FIRMPacket]:
        """
        Retrieve FIRMPacket objects parsed by the background thread.

        Args:
            block: If True, wait for at least one packet.
            max_items: Optional cap on number of packets returned.

        Returns:
            List of FIRMPacket objects.
        """
        firm_packets: list[FIRMPacket] = []

        if block:
            # Keep waiting until we successfully get a packet
            while not firm_packets:
                packet = self._packet_queue.get()
                firm_packets.append(packet)

        try:
            while max_items is None or len(firm_packets) < max_items:
                firm_packets.append(self._packet_queue.get_nowait())
        except queue.Empty:
            pass

        return firm_packets

    def _serial_reader(self):
        """Continuously read from serial port, parse packets, and enqueue them."""
        serial_port = self._serial_port
        stop = self._stop_event

        while not stop.is_set():
            # Get how many bytes are waiting
            try:
                number_of_bytes_in_buffer = serial_port.in_waiting
            except (OSError, serial.SerialException):
                break  # serial port error, exit thread
            # We either get how many bytes are waiting, or if there are no bytes we say to read
            # at least 1 byte, which will block until data arrives or timeout occurs.
            number_of_bytes_to_read = (
                number_of_bytes_in_buffer if number_of_bytes_in_buffer > 0 else 1
            )

            # Read the available bytes
            try:
                # This may block until data is available or timeout occurs
                new_bytes = serial_port.read(number_of_bytes_to_read)
            except (OSError, serial.SerialException):
                break

            if not new_bytes:
                continue  # timed out or no data yet

            # Parse as many packets as possible
            self._bytes_stored.extend(new_bytes)
            packets = self._parse_packets()

            # Add the new packets to the queue, dropping oldest if full
            for packet in packets:
                try:
                    self._packet_queue.put_nowait(packet)
                except queue.Full:
                    try:
                        _ = self._packet_queue.get_nowait()  # drop oldest
                        self._packet_queue.put_nowait(packet)
                    except queue.Empty:
                        pass  # rare race: ignore and continue

    def _parse_packets(self) -> list[FIRMPacket]:
        """Parse as many complete packets as possible and return FIRMPacket objects.
        Any leftover bytes for an incomplete packet are retained for the next read.

        Returns: list of FIRMPacket objects parsed.
        """
        packets: list[FIRMPacket] = []
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
            firm_packet = self._create_firm_packet(payload)  # returns FIRMPacket | None
            if firm_packet:
                packets.append(firm_packet)

            pos = crc_start + CRC_SIZE

        # Retain unparsed data, delete parsed bytes:
        self._bytes_stored = self._bytes_stored[pos:]

        return packets

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
