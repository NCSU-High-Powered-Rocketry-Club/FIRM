with open('LOG198.TXT', 'rb') as f:
    f.seek(234)
    
    # Read first 50 bytes raw
    data = f.read(50)
    
    print("Position analysis:")
    print("Offset | Hex  | Char")
    print("-------|------|-----")
    for i, b in enumerate(data[:50]):
        offset = 234 + i
        char = chr(b) if 32 <= b < 127 else '.'
        
        # Mark packet IDs
        if chr(b) in ['B', 'I', 'M']:
            print(f"{offset:6d} | 0x{b:02X} | '{chr(b)}' <-- PACKET ID")
        else:
            print(f"{offset:6d} | 0x{b:02X} | '{char}'")

print("\nBased on the structure:")
print("BMP581: ID(1) + ts(3) + data(6) = 10 bytes")
print("ICM45686: ID(1) + ts(3) + data(15) = 19 bytes")
print("MMC5983MA: ID(1) + ts(3) + data(7) = 11 bytes")
