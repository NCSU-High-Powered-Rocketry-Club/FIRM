#!/usr/bin/env python3
"""
Calculate what the CRC SHOULD be for the actual packet data
"""

def crc16_ccitt_kermit(data):
    """CRC-16-CCITT (KERMIT) implementation"""
    poly = 0x8408  # Reversed polynomial
    crc = 0x0000   # Initial value for KERMIT
    
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1
    return crc & 0xFFFF

# PACKET 1 actual data (first 56 bytes)
packet1_hex = "5A A5 34 00 D0 BC FA 41 2C FA C4 47 00 00 E5 3C 00 40 81 BD 00 0C 80 BF 65 71 21 3C 6E 88 2E 3B 57 A0 8B 3A E4 C1 C7 C4 D2 53 C5 C4 00 A1 29 C1 56 F1 46 E6 C1 42 62 40"
packet1_bytes = bytes.fromhex(packet1_hex.replace(" ", ""))

print("PACKET 1 Analysis")
print("=" * 70)
print(f"First 56 bytes (hex): {packet1_bytes.hex(' ').upper()}")
print(f"Length: {len(packet1_bytes)} bytes")
print()

# Calculate CRC
calculated_crc = crc16_ccitt_kermit(packet1_bytes)
print(f"Calculated CRC: 0x{calculated_crc:04X}")

# Extract received CRC (bytes 56-57)
received_crc_bytes = bytes([0x3C, 0x49])
received_crc_le = int.from_bytes(received_crc_bytes, 'little')
received_crc_be = int.from_bytes(received_crc_bytes, 'big')

print(f"Received CRC (little-endian): 0x{received_crc_le:04X}")
print(f"Received CRC (big-endian):    0x{received_crc_be:04X}")
print()

print("Match check:")
print(f"  Calculated == Received (LE): {calculated_crc == received_crc_le}")
print(f"  Calculated == Received (BE): {calculated_crc == received_crc_be}")
print()

# Let's also try XORing the result (sometimes CRCs are inverted)
crc_inverted = calculated_crc ^ 0xFFFF
print(f"Inverted CRC: 0x{crc_inverted:04X}")
print(f"  Inverted == Received (LE): {crc_inverted == received_crc_le}")
print(f"  Inverted == Received (BE): {crc_inverted == received_crc_be}")
