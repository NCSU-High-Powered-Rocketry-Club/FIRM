import os
import struct
import sys


# Trims a v1.2 binary log file to only include packets between two timestamps (seconds).
# Output keeps the entire header and only the packet section within the requested range.


# v1.2 header layout (matches current decoder.py logic)
HEADER_TEXT_SIZE = 14
UID_SIZE = 8
DEVICE_NAME_LEN = 32
COMM_SIZE = 4
FIRMWARE_VERSION_LEN = 8
FREQUENCY_LEN = 2
CAL_BYTES = 144
NUM_SCALE_FACTORS = 5
SCALE_FACTOR_BYTES = NUM_SCALE_FACTORS * 4

V12_TEXT = b"FIRM LOG v1.2\n"
CPU_HZ = 168e6


# packet sizes (payload bytes after id + timestamp)
BMP581_ID = ord("B")
ICM45686_ID = ord("I")
MMC5983MA_ID = ord("M")
BMP581_SIZE = 6
ICM45686_SIZE = 15
MMC5983MA_SIZE = 7
TIMESTAMP_BYTES = 4


def _read_exact(f, n: int) -> bytes:
    b = f.read(n)
    if len(b) != n:
        raise EOFError(f"Unexpected EOF while reading {n} bytes")
    return b


def _read_v12_header_bytes(f) -> bytes:
    header_text = _read_exact(f, HEADER_TEXT_SIZE)
    if header_text != V12_TEXT:
        raise ValueError("Input file is not v1.2 (expected 'FIRM LOG v1.2\\n')")

    uid_b = _read_exact(f, UID_SIZE)
    device_name_b = _read_exact(f, DEVICE_NAME_LEN)
    comms_b = _read_exact(f, COMM_SIZE)
    firmware_b = _read_exact(f, FIRMWARE_VERSION_LEN)
    frequency_b = _read_exact(f, FREQUENCY_LEN)

    header_before_padding = UID_SIZE + DEVICE_NAME_LEN + COMM_SIZE + FIRMWARE_VERSION_LEN + FREQUENCY_LEN
    padding_bytes = (8 - (header_before_padding % 8)) % 8
    padding_b = _read_exact(f, padding_bytes) if padding_bytes else b""

    calibration_b = _read_exact(f, CAL_BYTES)
    scale_factors_b = _read_exact(f, SCALE_FACTOR_BYTES)

    return (
        header_text
        + uid_b
        + device_name_b
        + comms_b
        + firmware_b
        + frequency_b
        + padding_b
        + calibration_b
        + scale_factors_b
    )


def trim_file(path: str, start_seconds: int, end_seconds: int) -> str:
    if start_seconds > end_seconds:
        raise ValueError("start_seconds must be <= end_seconds")

    out_path = os.path.join(os.path.dirname(path), f"trimmed_{os.path.basename(path)}")

    with open(path, "rb") as src, open(out_path, "wb") as dst:
        header_bytes = _read_v12_header_bytes(src)
        dst.write(header_bytes)

        timestamp_seconds = 0.0
        last_clock_count = 0

        in_range_started = False

        while True:
            id_b = src.read(1)
            if not id_b:
                break

            # Skip whitespace bytes (same as decoder behavior)
            if id_b[0] == 0:
                continue

            clock_count_bytes = src.read(TIMESTAMP_BYTES)
            if len(clock_count_bytes) != TIMESTAMP_BYTES:
                break

            clock_count = struct.unpack("<I", clock_count_bytes)[0]

            next_clock_count = clock_count
            if next_clock_count < last_clock_count:
                next_clock_count += (2**32)
            delta = next_clock_count - last_clock_count
            timestamp_seconds += delta / CPU_HZ
            last_clock_count = clock_count

            if id_b[0] == BMP581_ID:
                payload_len = BMP581_SIZE
            elif id_b[0] == ICM45686_ID:
                payload_len = ICM45686_SIZE
            elif id_b[0] == MMC5983MA_ID:
                payload_len = MMC5983MA_SIZE
            else:
                # if not an ID byte, most likely garbage data at end of file
                break

            payload = src.read(payload_len)
            if len(payload) != payload_len:
                break

            if timestamp_seconds < start_seconds:
                continue
            if timestamp_seconds > end_seconds:
                if in_range_started:
                    break
                continue

            in_range_started = True
            dst.write(id_b)
            dst.write(clock_count_bytes)
            dst.write(payload)

    return out_path


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: uv run file_trimmer.py <file> <start_seconds> <end_seconds>")
        sys.exit(1)

    path = sys.argv[1]
    if not os.path.exists(path):
        print("Invalid path to file")
        sys.exit(1)

    try:
        start_s = int(sys.argv[2])
        end_s = int(sys.argv[3])
    except ValueError:
        print("start_seconds and end_seconds must be integers")
        sys.exit(1)

    try:
        out_path = trim_file(path, start_s, end_s)
        print(out_path)
    except Exception as e:
        print("An error occurred: ", e)
        sys.exit(1)
