import struct

ADXL371_ID = ord('A')
TIMESTAMP_BYTES = 4

with open('LOG26.FRM', 'rb') as f:
    # Find the header size by looking for first packet
    # Use the same header parsing as decoder.py
    HEADER_SIZE_TEXT = 14
    HEADER_UID_SIZE = 8
    HEADER_DEVICE_NAME_LEN = 32
    HEADER_COMM_SIZE = 4
    FIRMWARE_VERSION_LEN = 8
    FREQUENCY_LEN = 2
    HEADER_CAL_SIZE = (3 + 9) * 3 * 4
    HEADER_NUM_SCALE_FACTORS = 5

    f.read(HEADER_SIZE_TEXT)
    f.read(HEADER_UID_SIZE)
    f.read(HEADER_DEVICE_NAME_LEN)
    f.read(HEADER_COMM_SIZE)
    f.read(FIRMWARE_VERSION_LEN)
    f.read(FREQUENCY_LEN)
    header_before_padding = HEADER_UID_SIZE + HEADER_DEVICE_NAME_LEN + HEADER_COMM_SIZE + FIRMWARE_VERSION_LEN + FREQUENCY_LEN
    padding_bytes = (8 - (header_before_padding % 8)) % 8
    if padding_bytes > 0:
        f.read(padding_bytes)
    f.read(HEADER_CAL_SIZE)
    f.read(HEADER_NUM_SCALE_FACTORS * 4)
    
    data_start = f.tell()
    print(f"Data starts at offset: {data_start}")
    
    # Now scan for ADXL371 packets
    adxl_packets = []
    pos = data_start
    f.seek(pos)
    raw = f.read(50000)  # Read a chunk
    
    # Find 'A' bytes and look at surrounding context
    print("\nFirst 20 ADXL371 ('A' = 0x41) packet locations and raw data:")
    count = 0
    i = 0
    while i < len(raw) and count < 20:
        if raw[i] == ADXL371_ID:
            # This might be an ADXL packet - show context
            ts_bytes = raw[i+1:i+5]
            data_bytes = raw[i+5:i+11]
            
            if len(data_bytes) == 6:
                ts = struct.unpack('<I', ts_bytes)[0]
                hex_data = ' '.join(f'{b:02X}' for b in data_bytes)
                hex_all = ' '.join(f'{b:02X}' for b in raw[i:i+11])
                
                # Current decode method: byte[0]<<4 | byte[1]>>4
                x_cur = (data_bytes[0] << 4) | (data_bytes[1] >> 4)
                y_cur = (data_bytes[2] << 4) | (data_bytes[3] >> 4)
                z_cur = (data_bytes[4] << 4) | (data_bytes[5] >> 4)
                
                # Apply twos complement
                def tc(val, bits):
                    if (val & (1 << (bits - 1))) != 0:
                        val = val - (1 << bits)
                    return val
                
                x_tc = tc(x_cur, 12)
                y_tc = tc(y_cur, 12)
                z_tc = tc(z_cur, 12)
                
                # Alt 1: Maybe the format is actually byte[1]<<8 | byte[0] (little-endian 16-bit, then >>4)
                x_alt1_raw = (data_bytes[1] << 8) | data_bytes[0]
                y_alt1_raw = (data_bytes[3] << 8) | data_bytes[2]
                z_alt1_raw = (data_bytes[5] << 8) | data_bytes[4]
                x_alt1 = tc(x_alt1_raw >> 4, 12)
                y_alt1 = tc(y_alt1_raw >> 4, 12)
                z_alt1 = tc(z_alt1_raw >> 4, 12)
                
                # Alt 2: big-endian 16-bit >> 4
                x_alt2_raw = (data_bytes[0] << 8) | data_bytes[1]
                y_alt2_raw = (data_bytes[2] << 8) | data_bytes[3]
                z_alt2_raw = (data_bytes[4] << 8) | data_bytes[5]
                x_alt2 = tc(x_alt2_raw >> 4, 12)
                y_alt2 = tc(y_alt2_raw >> 4, 12)
                z_alt2 = tc(z_alt2_raw >> 4, 12)
                
                # Alt 3: Current method but without twos complement (unsigned)
                x_alt3 = x_cur
                y_alt3 = y_cur
                z_alt3 = z_cur
                
                print(f"\n  Packet at offset {pos+i}: {hex_all}")
                print(f"    Data bytes: {hex_data}")
                print(f"    Current (b0<<4|b1>>4, tc12):  x={x_tc:6d}  y={y_tc:6d}  z={z_tc:6d}  -> x={x_tc/10.24:.2f}g  y={y_tc/10.24:.2f}g  z={z_tc/10.24:.2f}g")
                print(f"    Alt1 (LE 16>>4, tc12):        x={x_alt1:6d}  y={y_alt1:6d}  z={z_alt1:6d}  -> x={x_alt1/10.24:.2f}g  y={y_alt1/10.24:.2f}g  z={z_alt1/10.24:.2f}g")
                print(f"    Alt2 (BE 16>>4, tc12):        x={x_alt2:6d}  y={y_alt2:6d}  z={z_alt2:6d}  -> x={x_alt2/10.24:.2f}g  y={y_alt2/10.24:.2f}g  z={z_alt2/10.24:.2f}g")
                
                adxl_packets.append((x_tc, y_tc, z_tc))
                count += 1
                i += 11  # Skip past this packet
                continue
        i += 1
    
    if adxl_packets:
        print("\n\nSummary of current decode (first 20 packets):")
        xs = [p[0] for p in adxl_packets]
        ys = [p[1] for p in adxl_packets]
        zs = [p[2] for p in adxl_packets]
        print(f"  X range: {min(xs)} to {max(xs)}")
        print(f"  Y range: {min(ys)} to {max(ys)}")
        print(f"  Z range: {min(zs)} to {max(zs)}")
