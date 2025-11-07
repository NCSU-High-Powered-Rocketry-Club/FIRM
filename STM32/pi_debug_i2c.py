#!/usr/bin/env python3
"""
Debug script to dump raw I2C data and analyze CRC issues
"""

import struct
from smbus2 import SMBus, i2c_msg

I2C_ADDRESS = 0x48
PACKET_SIZE = 58

def crc16_ccitt(data):
    """Calculate CRC-16-CCITT (KERMIT) - matches STM32 implementation"""
    poly = 0x8408
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1
    return crc & 0xFFFF

def read_and_dump():
    """Read packet and dump all information"""
    try:
        with SMBus(1) as bus:
            # Read all 58 bytes
            msg = i2c_msg.read(I2C_ADDRESS, PACKET_SIZE)
            bus.i2c_rdwr(msg)
            data = bytes(msg)
            
            print("="*70)
            print("RAW DATA (58 bytes):")
            print("="*70)
            
            # Print hex dump
            for i in range(0, len(data), 16):
                hex_str = ' '.join(f'{b:02X}' for b in data[i:i+16])
                print(f"{i:04d}: {hex_str}")
            
            print("\n" + "="*70)
            print("PARSED STRUCTURE:")
            print("="*70)
            
            # Parse components
            header = struct.unpack('<H', data[0:2])[0]
            length = struct.unpack('<H', data[2:4])[0]
            
            print(f"Header:  0x{header:04X} (expected 0xA55A)")
            print(f"Length:  {length} (expected 52)")
            print()
            
            # Parse payload (11 floats + 1 double)
            payload_data = data[4:56]
            temp, pressure, ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack('<11f', payload_data[:44])
            timestamp = struct.unpack('<d', payload_data[44:52])[0]
            
            print(f"Temperature:  {temp:.2f} °C")
            print(f"Pressure:     {pressure:.2f} Pa")
            print(f"Accel:        ({ax:.3f}, {ay:.3f}, {az:.3f}) m/s²")
            print(f"Gyro:         ({gx:.3f}, {gy:.3f}, {gz:.3f}) rad/s")
            print(f"Mag:          ({mx:.3f}, {my:.3f}, {mz:.3f}) µT")
            print(f"Timestamp:    {timestamp:.6f} s")
            print()
            
            # CRC analysis
            received_crc = struct.unpack('<H', data[56:58])[0]
            calculated_crc = crc16_ccitt(data[:56])  # CRC over first 56 bytes
            
            print(f"Received CRC:    0x{received_crc:04X}")
            print(f"Calculated CRC:  0x{calculated_crc:04X}")
            print(f"CRC Valid:       {received_crc == calculated_crc}")
            
            if received_crc != calculated_crc:
                print("\n⚠️  CRC MISMATCH!")
                print(f"Difference: 0x{abs(received_crc - calculated_crc):04X}")
                
                # Try calculating CRC over different ranges to debug
                print("\nTrying different CRC ranges:")
                for end in [52, 54, 56, 58]:
                    test_crc = crc16_ccitt(data[:end])
                    match = "✓" if test_crc == received_crc else "✗"
                    print(f"  CRC of first {end:2d} bytes: 0x{test_crc:04X} {match}")
            else:
                print("\n✓ CRC VALID!")
            
            print("="*70)
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    print("I2C Debug - Reading from STM32 at address 0x{:02X}".format(I2C_ADDRESS))
    print()
    
    # Read a few packets
    for i in range(3):
        print(f"\n{'#'*70}")
        print(f"# PACKET {i+1}")
        print(f"{'#'*70}\n")
        read_and_dump()
        
        if i < 2:
            import time
            time.sleep(0.2)
