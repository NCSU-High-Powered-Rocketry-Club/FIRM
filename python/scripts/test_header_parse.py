import struct

# Constants  
HEADER_SIZE_TEXT = 14
HEADER_UID_SIZE = 8
HEADER_DEVICE_NAME_LEN = 32
HEADER_COMM_SIZE = 4
FIRMWARE_VERSION_LEN = 8
FREQUENCY_LEN = 2
HEADER_CAL_SIZE = (3 + 9) * 3 * 4
HEADER_NUM_SCALE_FACTORS = 5

with open('LOG198.TXT', 'rb') as f:
    # Read header text
    header_text = f.read(HEADER_SIZE_TEXT)
    print(f"Header text ({HEADER_SIZE_TEXT} bytes): {header_text}")
    
    # Read UID
    uid_b = f.read(HEADER_UID_SIZE)
    print(f"UID ({HEADER_UID_SIZE} bytes): {uid_b.hex()}")
    
    # Read device name
    device_name_b = f.read(HEADER_DEVICE_NAME_LEN)
    print(f"Device name ({HEADER_DEVICE_NAME_LEN} bytes): {device_name_b}")
    
    # Read comms
    comms_b = f.read(HEADER_COMM_SIZE)
    print(f"Comms ({HEADER_COMM_SIZE} bytes): {comms_b.hex()}")
    
    # Read firmware
    firmware_b = f.read(FIRMWARE_VERSION_LEN)
    print(f"Firmware ({FIRMWARE_VERSION_LEN} bytes): {firmware_b}")
    
    # Read frequency
    frequency_b = f.read(FREQUENCY_LEN)
    print(f"Frequency ({FREQUENCY_LEN} bytes): {frequency_b.hex()}")
    
    # Calculate padding
    header_before_padding = HEADER_UID_SIZE + HEADER_DEVICE_NAME_LEN + HEADER_COMM_SIZE + FIRMWARE_VERSION_LEN + FREQUENCY_LEN
    padding_bytes = (8 - (header_before_padding % 8)) % 8
    print(f"\nPadding calculation:")
    print(f"  Header before padding: {header_before_padding}")
    print(f"  Padding bytes needed: {padding_bytes}")
    
    # Read padding
    if padding_bytes > 0:
        padding_b = f.read(padding_bytes)
        print(f"  Padding bytes: {padding_b.hex()}")
    
    # Read calibration
    calibration_b = f.read(HEADER_CAL_SIZE)
    print(f"\nCalibration ({HEADER_CAL_SIZE} bytes):")
    # Parse as floats
    cals = struct.unpack('<' + 'f' * (HEADER_CAL_SIZE // 4), calibration_b)
    print(f"  First 3 values: {cals[0:3]}")
    
    # Read scale factors
    scale_factor_b = f.read(HEADER_NUM_SCALE_FACTORS * 4)
    print(f"\nScale factors ({HEADER_NUM_SCALE_FACTORS * 4} bytes): {scale_factor_b.hex()}")
    scale_factors = struct.unpack('<fffff', scale_factor_b)
    print(f"  Values: {scale_factors}")
    
    # Check position
    file_pos = f.tell()
    print(f"\nFile position after header: {file_pos}")
    
    # Read next byte (should be packet ID)
    next_byte = f.read(1)
    print(f"Next byte (should be packet ID): {next_byte.hex()} ({chr(next_byte[0]) if next_byte[0] < 128 else '?'})")
