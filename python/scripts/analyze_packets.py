BMP581_ID = ord('B')
ICM45686_ID = ord('I')
MMC5983MA_ID = ord('M')

with open('LOG198.TXT', 'rb') as f:
    # Read all 500 bytes starting from offset 234
    f.seek(234)
    data = f.read(500)
    
    print(f"First 100 bytes from offset 234:")
    hex_str = ' '.join(f'{b:02X}' for b in data[:100])
    print(hex_str)
    
    print(f"\nLooking for packet IDs in first 100 bytes:")
    for i, b in enumerate(data[:100]):
        if b == BMP581_ID:
            print(f"'B' at offset {234 + i}")
        elif b == ICM45686_ID:
            print(f"'I' at offset {234 + i}")
        elif b == MMC5983MA_ID:
            print(f"'M' at offset {234 + i}")
