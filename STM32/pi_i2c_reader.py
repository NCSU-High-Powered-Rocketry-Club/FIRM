#!/usr/bin/env python3
"""
I2C Reader for STM32 Sensor Data
Reads SerializedPacket_t from STM32 I2C slave at address 0x48
"""

import struct
import time
from typing import Optional, Tuple
from smbus2 import SMBus, i2c_msg


class CRC16:
    """CRC-16-CCITT (KERMIT variant) calculator"""

    _TABLE = [
        0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
        0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
        0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
        0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
        0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
        0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
        0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
        0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
        0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
        0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
        0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
        0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
        0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
        0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
        0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
        0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
        0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
        0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
        0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
        0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
        0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
        0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
        0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
        0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
        0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
        0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
        0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
        0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
        0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
        0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
        0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
        0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
    ]

    @classmethod
    def calculate(cls, data: bytes) -> int:
        """Calculate CRC-16-CCITT (KERMIT) for given data"""
        crc = 0
        for byte in data:
            idx = (crc ^ byte) & 0xFF
            crc = (cls._TABLE[idx] ^ (crc >> 8)) & 0xFFFF
        return crc


class SensorPacket:
    """Parsed sensor data packet from STM32"""
    
    def __init__(self):
        self.header: int = 0
        self.length: int = 0
        self.timestamp: int = 0
        self.pressure_raw: int = 0
        self.pressure_frac: int = 0
        self.accel_x: float = 0.0
        self.accel_y: float = 0.0
        self.accel_z: float = 0.0
        self.gyro_x: float = 0.0
        self.gyro_y: float = 0.0
        self.gyro_z: float = 0.0
        self.mag_x: float = 0.0
        self.mag_y: float = 0.0
        self.mag_z: float = 0.0
        self.temperature: float = 0.0
        self.altitude: float = 0.0
        self.reserved: int = 0
        self.crc: int = 0
        self.crc_valid: bool = False
    
    def __str__(self) -> str:
        return (
            f"SensorPacket(\n"
            f"  timestamp={self.timestamp:.3f}s,\n"
            f"  temperature={self.temperature:.2f}°C,\n"
            f"  pressure={self.pressure_raw:.2f} Pa,\n"
            f"  accel=({self.accel_x:.3f}, {self.accel_y:.3f}, {self.accel_z:.3f}) m/s²,\n"
            f"  gyro=({self.gyro_x:.3f}, {self.gyro_y:.3f}, {self.gyro_z:.3f}) rad/s,\n"
            f"  mag=({self.mag_x:.3f}, {self.mag_y:.3f}, {self.mag_z:.3f}) µT,\n"
            f"  crc_valid={self.crc_valid}\n"
            f")"
        )


class STM32I2CReader:
    """I2C reader for STM32 sensor data"""
    
    PACKET_SIZE = 58  # Total packet size in bytes
    HEADER_MAGIC = 0xA55A  # Expected header value
    I2C_ADDRESS = 0x48  # STM32 I2C slave address
    
    def __init__(self, bus_number: int = 1):
        """
        Initialize I2C reader
        
        Args:
            bus_number: I2C bus number (default 1 for Raspberry Pi)
        """
        self.bus_number = bus_number
        self.crc_calculator = CRC16()
    
    def read_packet(self, timeout: float = 1.0) -> Optional[SensorPacket]:
        """
        Read a complete sensor packet from STM32
        
        Args:
            timeout: Maximum time to wait for valid packet (seconds)
        
        Returns:
            SensorPacket object if successful, None if failed
        """
        try:
            with SMBus(self.bus_number) as bus:
                # Read all 58 bytes at once
                msg = i2c_msg.read(self.I2C_ADDRESS, self.PACKET_SIZE)
                bus.i2c_rdwr(msg)
                data = bytes(msg)
                
                # Parse the packet
                return self._parse_packet(data)
                
        except Exception as e:
            print(f"I2C read error: {e}")
            return None
    
    def read_packet_chunked(self, chunk_size: int = 16) -> Optional[SensorPacket]:
        """
        Read packet in chunks (more reliable for some I2C controllers)
        
        Args:
            chunk_size: Number of bytes to read per transaction
        
        Returns:
            SensorPacket object if successful, None if failed
        """
        try:
            with SMBus(self.bus_number) as bus:
                data = bytearray()
                
                # Read in chunks
                for offset in range(0, self.PACKET_SIZE, chunk_size):
                    remaining = min(chunk_size, self.PACKET_SIZE - offset)
                    msg = i2c_msg.read(self.I2C_ADDRESS, remaining)
                    bus.i2c_rdwr(msg)
                    data.extend(bytes(msg))
                    
                    # Small delay between chunks
                    if offset + remaining < self.PACKET_SIZE:
                        time.sleep(0.001)
                
                # Parse the packet
                return self._parse_packet(bytes(data))
                
        except Exception as e:
            print(f"I2C chunked read error: {e}")
            return None
    
    def _parse_packet(self, data: bytes) -> Optional[SensorPacket]:
        """
        Parse raw bytes into SensorPacket
        
        Packet structure (58 bytes total):
        - header: uint16_t (2 bytes) - 0xA55A
        - length: uint16_t (2 bytes) - payload length (52)
        - CalibratedDataPacket_t payload (52 bytes):
          - temperature: float (4 bytes)
          - pressure: float (4 bytes)
          - accel_x/y/z: float (4 bytes each, 12 total)
          - angular_rate_x/y/z: float (4 bytes each, 12 total)
          - magnetic_field_x/y/z: float (4 bytes each, 12 total)
          - timestamp_sec: double (8 bytes)
        - crc: uint16_t (2 bytes)
        """
        if len(data) != self.PACKET_SIZE:
            print(f"Invalid packet size: {len(data)} (expected {self.PACKET_SIZE})")
            return None
        
        packet = SensorPacket()
        
        try:
            # Unpack the packet (little-endian format)
            # Format: HH (4) + 11 floats (44) + 1 double (8) + H (2) = 58 bytes
            # Structure: header, length, temp, pressure, accel(3), gyro(3), mag(3), timestamp, crc
            unpacked = struct.unpack('<HH 11f d H', data)
            
            packet.header = unpacked[0]
            packet.length = unpacked[1]
            packet.temperature = unpacked[2]
            packet.pressure_raw = int(unpacked[3])  # Store as integer for display
            packet.pressure_frac = 0  # Not used in this format
            packet.accel_x = unpacked[4]
            packet.accel_y = unpacked[5]
            packet.accel_z = unpacked[6]
            packet.gyro_x = unpacked[7]
            packet.gyro_y = unpacked[8]
            packet.gyro_z = unpacked[9]
            packet.mag_x = unpacked[10]
            packet.mag_y = unpacked[11]
            packet.mag_z = unpacked[12]
            packet.timestamp = unpacked[13]  # timestamp_sec (double)
            packet.altitude = 0.0  # Not in this packet format
            packet.reserved = 0
            packet.crc = unpacked[14]
            
            # Verify header
            if packet.header != self.HEADER_MAGIC:
                print(f"Invalid header: 0x{packet.header:04X} (expected 0x{self.HEADER_MAGIC:04X})")
                return None
            
            # Verify CRC (calculate over all data except the CRC itself)
            calculated_crc = self.crc_calculator.calculate(data[:-2])
            packet.crc_valid = (calculated_crc == packet.crc)
            
            if not packet.crc_valid:
                print(f"CRC mismatch: calculated=0x{calculated_crc:04X}, received=0x{packet.crc:04X}")
            
            return packet
            
        except struct.error as e:
            print(f"Packet parsing error: {e}")
            return None
    
    def read_continuous(self, interval: float = 0.1, callback=None):
        """
        Continuously read packets at specified interval
        
        Args:
            interval: Time between reads in seconds
            callback: Optional function to call with each packet (receives SensorPacket)
        """
        print(f"Starting continuous read from I2C address 0x{self.I2C_ADDRESS:02X}")
        print(f"Reading every {interval}s... Press Ctrl+C to stop")
        
        try:
            while True:
                packet = self.read_packet()
                
                if packet:
                    if callback:
                        callback(packet)
                    else:
                        print(packet)
                        print("-" * 60)
                
                time.sleep(interval)
                
        except KeyboardInterrupt:
            print("\nStopped by user")


def main():
    """Example usage"""
    reader = STM32I2CReader(bus_number=1)
    
    print("Reading single packet:")
    packet = reader.read_packet()
    if packet:
        print(packet)
        print(f"\nRaw hex data would be: {packet.header:04x}{packet.length:04x}...")
    else:
        print("Failed to read packet")
    
    print("\n" + "="*60)
    print("Starting continuous read (Ctrl+C to stop)...")
    print("="*60 + "\n")
    
    # Read continuously
    reader.read_continuous(interval=0.1)


if __name__ == "__main__":
    main()
