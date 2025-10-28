"""Parser for FIRM data packets from a serial interface."""

import queue
import struct
import threading

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
from .packets import FIRMPacket


class FIRM:
    """Parser for FIRM data packets from a serial interface.

    TODO: Write more docs here.

    """

    __slots__ = (
        "_bytes_stored",
        "_packet_queue",
        "_serial_port",
        "_serial_reader_thread",
        "_stop_event",
        "_struct",
        "_one_shot_mode",
        "_most_recent_packet",
    )

    def __init__(self, port: str, baudrate: int, one_shot_mode: bool = False):
        self._serial_port = serial.Serial(port, baudrate)
        self._one_shot_mode = one_shot_mode
        self._bytes_stored = bytearray()
        self._struct = struct.Struct("<fffffffffffxxxxd")
        self._serial_reader_thread = None
        self._stop_event = threading.Event()
        self._packet_queue: queue.Queue[FIRMPacket] = queue.Queue(maxsize=8192)
        self._most_recent_packet: FIRMPacket | None = None

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
        if (not self._one_shot_mode) and (self._serial_reader_thread is None or not self._serial_reader_thread.is_alive()):
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

    def get_most_recent_data_packet(self) -> FIRMPacket:
        """
        Always attempt to parse and return the most recent complete packet.
        If there is not yet enough data in the serial buffer to form a packet,
        this method will block until one can be parsed.
        """
        if not self._one_shot_mode:
            raise RuntimeError("get_most_recent_data_packet() can only be used in one-shot mode.")

        while True:
            # Read any newly arrived bytes from the serial port
            bytes_waiting = self._serial_port.in_waiting
            if bytes_waiting > 0:
                new_bytes = self._serial_port.read(bytes_waiting)
                if new_bytes:
                    self._bytes_stored.extend(new_bytes)

            # Gets the newest header position
            header_position = self._bytes_stored.rfind(START_BYTE)
            if header_position == -1:
                # No header yet; block until more data arrives, this will really only happen once at
                # the start
                self._bytes_stored.extend(self._serial_port.read(1))
                continue

            # Try to parse the most recent complete packet, walking backward if needed
            while header_position != -1:
                packet_end = header_position + FULL_PACKET_SIZE

                # Wait until we have enough bytes for a full packet
                while len(self._bytes_stored) < packet_end:
                    self._bytes_stored.extend(
                        self._serial_port.read(packet_end - len(self._bytes_stored) or 1)
                    )

                view = memoryview(self._bytes_stored)

                status, length_start_pos, crc_end_pos, firm_packet = self._try_build_packet_at(
                    view, header_position
                )

                if status == "ok":
                    self._most_recent_packet = firm_packet
                    pos = crc_end_pos  # identical trimming semantics
                    self._bytes_stored = self._bytes_stored[pos:]
                    return firm_packet

                # If it was not a valid packet, look for an earlier header
                header_position = self._bytes_stored.rfind(START_BYTE, 0, header_position)

            # If no valid header worked, read one more byte and try again
            self._bytes_stored.extend(self._serial_port.read(1))


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
        if self._one_shot_mode:
            raise RuntimeError("get_data_packets() cannot be used in one-shot mode.")

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

            status, length_start_pos, crc_end_pos, firm_packet = self._try_build_packet_at(view, header_pos)

            if status == "ok":
                packets.append(firm_packet)
                pos = crc_end_pos
            else:
                pos = length_start_pos
                continue

        # Retain unparsed data, delete parsed bytes:
        self._bytes_stored = self._bytes_stored[pos:]

        return packets

    def _try_build_packet_at(
            self,
            data_view: memoryview,
            header_position: int,
    ) -> tuple[str, int, int, FIRMPacket | None]:
        """
        Attempt to parse a FIRMPacket at header_position.

        Returns:
            (status, length_start_position, crc_end_position, packet_or_none)
            - status: "ok", "bad_length", "bad_crc", "unpack_error"
            - length_start_position: header_position + HEADER_SIZE
            - crc_end_position: (crc_start + CRC_SIZE) when status == "ok", else undefined
            - packet_or_none: FIRMPacket when status == "ok", else None

        NOTE: This function does NOT change any state; callers decide how to advance/truncate.
        """
        length_start_position = header_position + HEADER_SIZE
        payload_length = int.from_bytes(
            data_view[length_start_position:length_start_position + LENGTH_FIELD_SIZE],
            "little",
        )

        if payload_length != PAYLOAD_LENGTH:
            return "bad_length", length_start_position, 0, None

        payload_start_position = (
                length_start_position + LENGTH_FIELD_SIZE + PADDING_SIZE
        )
        crc_start_position = payload_start_position + payload_length

        if not self._verify_crc(data_view, header_position, crc_start_position):
            return "bad_crc", length_start_position, 0, None

        payload_bytes = bytes(data_view[payload_start_position:crc_start_position])
        firm_packet = self._create_firm_packet(payload_bytes)
        if firm_packet is None:
            return "unpack_error", length_start_position, 0, None

        crc_end_position = crc_start_position + CRC_SIZE
        return "ok", length_start_position, crc_end_position, firm_packet


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
