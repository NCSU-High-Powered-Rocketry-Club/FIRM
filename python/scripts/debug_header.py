HEADER_SIZE_TEXT = 14
HEADER_UID_SIZE = 8
HEADER_DEVICE_NAME_LEN = 32
HEADER_COMM_SIZE = 4
FIRMWARE_VERSION_LEN = 8
FREQUENCY_LEN = 2

header_before_padding = HEADER_UID_SIZE + HEADER_DEVICE_NAME_LEN + HEADER_COMM_SIZE + FIRMWARE_VERSION_LEN + FREQUENCY_LEN
print(f'Header before padding: {header_before_padding} bytes')
print(f'Header before padding % 8: {header_before_padding % 8}')

padding_bytes = (8 - (header_before_padding % 8)) % 8
print(f'Padding bytes needed: {padding_bytes}')

total_header_offset = HEADER_SIZE_TEXT + header_before_padding + padding_bytes
print(f'Total offset before calibration: {total_header_offset}')

HEADER_CAL_SIZE = (3 + 9) * 3 * 4
print(f'Calibration size: {HEADER_CAL_SIZE}')
print(f'Total offset before scale factors: {total_header_offset + HEADER_CAL_SIZE}')

HEADER_NUM_SCALE_FACTORS = 5
scale_factor_size = HEADER_NUM_SCALE_FACTORS * 4
print(f'Scale factors size: {scale_factor_size}')
print(f'Total header size: {total_header_offset + HEADER_CAL_SIZE + scale_factor_size}')
