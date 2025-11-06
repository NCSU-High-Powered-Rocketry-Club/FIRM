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
    
    def __init__(self):
        self.poly = 0x8408  # Reversed polynomial for KERMIT
        self.table = self._build_table()
    
    def _build_table(self):
        """Build CRC lookup table"""
        table = []
        for byte in range(256):
            crc = 0
            for _ in range(8):
                if (byte ^ crc) & 1:
                    crc = (crc >> 1) ^ self.poly
                else:
                    crc >>= 1
                byte >>= 1
            table.append(crc)
        return table
    
    def calculate(self, data: bytes) -> int:
        """Calculate CRC-16-CCITT (KERMIT) for given data"""
        crc = 0
        for byte in data:
            crc = (crc >> 8) ^ self.table[(crc ^ byte) & 0xFF]
        return crc & 0xFFFF


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
            f"  timestamp={self.timestamp},\n"
            f"  pressure={self.pressure_raw + self.pressure_frac / 1e9:.2f} Pa,\n"
            f"  accel=({self.accel_x:.3f}, {self.accel_y:.3f}, {self.accel_z:.3f}) m/s²,\n"
            f"  gyro=({self.gyro_x:.3f}, {self.gyro_y:.3f}, {self.gyro_z:.3f}) rad/s,\n"
            f"  mag=({self.mag_x:.3f}, {self.mag_y:.3f}, {self.mag_z:.3f}) µT,\n"
            f"  temperature={self.temperature:.2f}°C,\n"
            f"  altitude={self.altitude:.2f}m,\n"
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
        - length: uint16_t (2 bytes) - payload length (56)
        - timestamp: uint64_t (8 bytes)
        - pressure_raw: uint32_t (4 bytes)
        - pressure_frac: uint8_t (1 byte)
        - accel_x/y/z: float (4 bytes each, 12 total)
        - gyro_x/y/z: float (4 bytes each, 12 total)
        - mag_x/y/z: float (4 bytes each, 12 total)
        - temperature: double (8 bytes)
        - altitude: float (4 bytes)
        - reserved: uint32_t (4 bytes)
        - crc: uint16_t (2 bytes)
        """
        if len(data) != self.PACKET_SIZE:
            print(f"Invalid packet size: {len(data)} (expected {self.PACKET_SIZE})")
            return None
        
        packet = SensorPacket()
        
        try:
            # Unpack the packet (little-endian format)
            # Format string: H=uint16, Q=uint64, I=uint32, B=uint8, f=float, d=double
            unpacked = struct.unpack('<HH Q IB fff fff fff d f I H', data)
            
            packet.header = unpacked[0]
            packet.length = unpacked[1]
            packet.timestamp = unpacked[2]
            packet.pressure_raw = unpacked[3]
            packet.pressure_frac = unpacked[4]
            packet.accel_x = unpacked[5]
            packet.accel_y = unpacked[6]
            packet.accel_z = unpacked[7]
            packet.gyro_x = unpacked[8]
            packet.gyro_y = unpacked[9]
            packet.gyro_z = unpacked[10]
            packet.mag_x = unpacked[11]
            packet.mag_y = unpacked[12]
            packet.mag_z = unpacked[13]
            packet.temperature = unpacked[14]
            packet.altitude = unpacked[15]
            packet.reserved = unpacked[16]
            packet.crc = unpacked[17]
            
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
