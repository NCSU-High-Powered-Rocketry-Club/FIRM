#!/usr/bin/env python3
"""
Verify that Python and STM32 CRC implementations produce the same result
"""

def python_crc16_ccitt(data):
    """Python implementation"""
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

# Test with simple known data
test_cases = [
    bytes([0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39]),  # "123456789"
    bytes([0x00, 0x00, 0x00, 0x00]),
    bytes([0xFF, 0xFF, 0xFF, 0xFF]),
    bytes([0x5A, 0xA5, 0x34, 0x00]),  # Header + Length from actual packet
]

print("Testing CRC implementation:")
print()

for i, data in enumerate(test_cases):
    crc = python_crc16_ccitt(data)
    print(f"Test {i+1}: {data.hex(' ').upper()}")
    print(f"  CRC: 0x{crc:04X}")
    print()

# The standard CRC-16-CCITT (KERMIT) result for "123456789" should be 0x2189
expected = 0x2189
actual = python_crc16_ccitt(test_cases[0])
print(f"Standard test vector:")
print(f"  Input: '123456789'")
print(f"  Expected CRC-16-CCITT (KERMIT): 0x{expected:04X}")
print(f"  Actual:                         0x{actual:04X}")
print(f"  Match: {expected == actual}")
