#!/usr/bin/env python3
"""Test CRC calculation on the actual packet data"""

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

# Packet 1 raw data (first 56 bytes, excluding CRC)
packet1 = bytes.fromhex(
    "5A A5 34 00 D0 BC FA 41 2C FA C4 47 00 00 E5 3C "
    "00 40 81 BD 00 0C 80 BF 65 71 21 3C 6E 88 2E 3B "
    "57 A0 8B 3A E4 C1 C7 C4 D2 53 C5 C4 00 A1 29 C1 "
    "56 F1 46 E6 C1 42 62 40"
)

print(f"Packet length: {len(packet1)} bytes")
print(f"Packet (hex): {packet1.hex(' ')}")
print()

# Calculate CRC
calc_crc = crc16_ccitt(packet1)
print(f"Calculated CRC: 0x{calc_crc:04X}")
print(f"  Little-endian bytes: {calc_crc & 0xFF:02X} {(calc_crc >> 8) & 0xFF:02X}")
print()

# Received CRC from packet
received_crc = 0x493C
print(f"Received CRC:   0x{received_crc:04X}")
print(f"  Little-endian bytes: {received_crc & 0xFF:02X} {(received_crc >> 8) & 0xFF:02X}")
print()

# Check if it's an endianness swap
received_swapped = ((received_crc & 0xFF) << 8) | ((received_crc >> 8) & 0xFF)
print(f"Received CRC (byte-swapped): 0x{received_swapped:04X}")
print(f"  Match? {calc_crc == received_swapped}")
