#!/usr/bin/env python3
"""
Simple I2C reader for STM32 sensor data
Reads 58-byte packets with CRC validation
"""

import struct
import time
from smbus2 import SMBus, i2c_msg

# Configuration
I2C_ADDRESS = 0x48
PACKET_SIZE = 58

class CRC16:
    """CRC-16-CCITT (KERMIT variant) calculator"""
    
    @staticmethod
    def calculate(data):
        """Calculate CRC-16-CCITT (KERMIT) over data bytes"""
        poly = 0x8408  # Reversed polynomial
        crc = 0x0000   # Initial value
        
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ poly
                else:
                    crc >>= 1
        
        return crc & 0xFFFF

class SensorPacket:
    """Represents a sensor data packet from STM32"""
    
    def __init__(self, data):
        """Parse 58-byte packet"""
        if len(data) != PACKET_SIZE:
            raise ValueError(f"Expected {PACKET_SIZE} bytes, got {len(data)}")
        
        # Parse header and length
        self.header = struct.unpack('<H', data[0:2])[0]
        self.length = struct.unpack('<H', data[2:4])[0]
        
        # Parse payload (11 floats + 1 double = 52 bytes)
        payload = struct.unpack('<11fd', data[4:56])
        
        self.temperature = payload[0]      # °C
        self.pressure = payload[1]         # Pa
        self.accel_x = payload[2]          # m/s²
        self.accel_y = payload[3]
        self.accel_z = payload[4]
        self.gyro_x = payload[5]           # rad/s
        self.gyro_y = payload[6]
        self.gyro_z = payload[7]
        self.mag_x = payload[8]            # µT
        self.mag_y = payload[9]
        self.mag_z = payload[10]
        self.timestamp = payload[11]       # seconds
        
        # Parse CRC
        self.received_crc = struct.unpack('<H', data[56:58])[0]
        
        # Validate CRC
        self.calculated_crc = CRC16.calculate(data[:56])
        self.crc_valid = (self.received_crc == self.calculated_crc)
    
    def __str__(self):
        """String representation"""
        return (
            f"Temperature: {self.temperature:.2f} °C\n"
            f"Pressure:    {self.pressure:.2f} Pa\n"
            f"Accel:       ({self.accel_x:.3f}, {self.accel_y:.3f}, {self.accel_z:.3f}) m/s²\n"
            f"Gyro:        ({self.gyro_x:.3f}, {self.gyro_y:.3f}, {self.gyro_z:.3f}) rad/s\n"
            f"Mag:         ({self.mag_x:.3f}, {self.mag_y:.3f}, {self.mag_z:.3f}) µT\n"
            f"Timestamp:   {self.timestamp:.6f} s\n"
            f"CRC Valid:   {self.crc_valid} (received=0x{self.received_crc:04X}, calculated=0x{self.calculated_crc:04X})"
        )

class STM32I2CReader:
    """I2C reader for STM32 sensor packets"""
    
    def __init__(self, bus_number=1, address=I2C_ADDRESS):
        self.bus_number = bus_number
        self.address = address
    
    def read_packet(self):
        """Read one packet from the STM32"""
        try:
            with SMBus(self.bus_number) as bus:
                msg = i2c_msg.read(self.address, PACKET_SIZE)
                bus.i2c_rdwr(msg)
                data = bytes(msg)
                return SensorPacket(data)
        except Exception as e:
            print(f"Error reading I2C: {e}")
            return None
    
    def read_continuous(self, interval=0.1, count=None):
        """
        Read packets continuously
        
        Args:
            interval: Time between reads in seconds
            count: Number of packets to read (None for infinite)
        """
        packet_num = 0
        
        try:
            while count is None or packet_num < count:
                packet = self.read_packet()
                
                if packet:
                    print(f"\n{'='*70}")
                    print(f"Packet #{packet_num + 1}")
                    print('='*70)
                    print(packet)
                    
                    if not packet.crc_valid:
                        print("\n⚠️  WARNING: CRC validation failed!")
                    
                    packet_num += 1
                    
                    if count is None or packet_num < count:
                        time.sleep(interval)
                else:
                    print("Failed to read packet, retrying...")
                    time.sleep(1)
                    
        except KeyboardInterrupt:
            print("\n\nStopped by user")
            print(f"Read {packet_num} packets")

def main():
    """Main function"""
    print("STM32 I2C Sensor Reader")
    print(f"Reading from address 0x{I2C_ADDRESS:02X}")
    print("Press Ctrl+C to stop\n")
    
    reader = STM32I2CReader()
    
    # Read packets continuously (update every 0.2 seconds)
    reader.read_continuous(interval=0.2)

if __name__ == "__main__":
    main()
