"""Parser for FIRM data packets from a serial interface."""

import queue
import struct
import threading

import serial

from ._constants import (
    CRC16_TABLE,
    CRC_SIZE,
    FULL_PACKET_SIZE,
    GRAVITY_METERS_PER_SECONDS_SQUARED,
    HEADER_SIZE,
    LENGTH_FIELD_SIZE,
    PADDING_SIZE,
    PAYLOAD_LENGTH,
    START_BYTE,
)
from ._packets import FIRMPacket


class FIRM:
    """
    Parser for FIRM data packets from a serial interface.

    Args:
        port (str): Serial port to connect to (e.g., "/dev/ttyACM0" or "COM3").
        baudrate (int): Baud rate for the serial connection
    """

    __slots__ = (
        "_bytes_stored",
        "_packet_queue",
        "_serial_port",
        "_serial_reader_thread",
        "_stop_event",
        "_struct",
    )

    def __init__(self, port: str, baudrate: int = 115_200):
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
        """Open serial and prepare for parsing packets by spawning a new thread.

        Raises:
            TimeoutError: If no packets are received within 1 second of initialization.
        """
        if not self._serial_port.is_open:
            self._serial_port.open()
        self._bytes_stored.clear()
        self._stop_event.clear()
        if self._serial_reader_thread is None or not self._serial_reader_thread.is_alive():
            self._serial_reader_thread = threading.Thread(
                target=self._serial_reader, name="Packet-Reader-Thread", daemon=True
            )
            self._serial_reader_thread.start()

        # Check if the serial port is returning packets. This will raise TimeoutError if not.
        self.get_data_packets(timeout=1.0)

    def close(self):
        """Close the serial port, and stop the packet reader thread."""
        self._stop_event.set()
        if self._serial_reader_thread is not None:
            self._serial_reader_thread.join(timeout=1.0)
            self._serial_reader_thread = None
        if self._serial_port.is_open:
            self._serial_port.close()

    def get_number_of_available_packets(self) -> int:
        """
        Get the number of FIRMPacket objects currently available in the queue.

        Returns:
            Number of available FIRMPacket objects.
        """
        return self._packet_queue.qsize()

    def get_most_recent_data_packet(self) -> FIRMPacket:
        """
        Retrieve the most recent FIRMPacket object parsed.
        """
        return self.get_data_packets(block=True)[-1]

    def get_data_packets(
        self,
        block: bool = True,
        timeout: float = 0.0,
    ) -> list[FIRMPacket]:
        """
        Retrieve FIRMPacket objects parsed by the background thread.

        Args:
            block: If True, wait for at least one packet.
            timeout: Maximum time to wait in seconds for a packet if `block` is ``True``.

        Returns:
            List of FIRMPacket objects.

        Raises:
            TimeoutError: If no packets are available within the specified timeout.
        """
        firm_packets: list[FIRMPacket] = []

        if block:
            # Keep waiting until we successfully get a packet
            while not firm_packets:
                try:
                    packet = self._packet_queue.get(timeout=timeout)
                except queue.Empty as e:
                    raise TimeoutError from e
                firm_packets.append(packet)

        while self._packet_queue.qsize() > 0:
            firm_packets.append(self._packet_queue.get_nowait())

        return firm_packets

    def _serial_reader(self):
        """Continuously read from serial port, parse packets, and enqueue them."""
        while not self._stop_event.is_set():
            new_bytes = self._serial_port.read(self._serial_port.in_waiting)
            # Parse as many packets as possible
            self._bytes_stored.extend(new_bytes)
            packets = self._parse_packets()

            # Add the new packets to the queue
            for packet in packets:
                self._packet_queue.put(packet)

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
            packets.append(self._create_firm_packet(payload))

            pos = crc_start + CRC_SIZE

        # Retain unparsed data, delete parsed bytes:
        self._bytes_stored = self._bytes_stored[pos:]

        return packets

    def _create_firm_packet(self, payload: bytes) -> FIRMPacket:
        """Unpack payload and create a single unified FIRMPacket."""

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
        ) = self._struct.unpack(payload)

        return FIRMPacket(
            timestamp_seconds=timestamp_seconds,
            # Convert acceleration from g to m/s²
            accel_x_meters_per_s2=accel_x_meters_per_s2 * GRAVITY_METERS_PER_SECONDS_SQUARED,
            accel_y_meters_per_s2=accel_y_meters_per_s2 * GRAVITY_METERS_PER_SECONDS_SQUARED,
            accel_z_meters_per_s2=accel_z_meters_per_s2 * GRAVITY_METERS_PER_SECONDS_SQUARED,
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
