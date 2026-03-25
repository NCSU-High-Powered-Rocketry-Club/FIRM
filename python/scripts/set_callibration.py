from __future__ import annotations

import argparse
import struct
from pathlib import Path

import yaml

HEADER_SIZE_TEXT = 14
HEADER_UID_SIZE = 8
HEADER_DEVICE_NAME_LEN = 32
HEADER_COMM_SIZE = 4
FIRMWARE_VERSION_LEN = 8
FREQUENCY_LEN = 2

CAL_FLOATS_PER_SENSOR = 12
CAL_TOTAL_FLOATS = CAL_FLOATS_PER_SENSOR * 4
CAL_BLOCK_SIZE_BYTES = CAL_TOTAL_FLOATS * 4

ACCEL_BASE = 0
GYRO_BASE = 12
MAG_BASE = 24
HIGH_G_BASE = 36


def _header_padding_size() -> int:
    header_before_padding = (
        HEADER_UID_SIZE + HEADER_DEVICE_NAME_LEN + HEADER_COMM_SIZE + FIRMWARE_VERSION_LEN + FREQUENCY_LEN
    )
    return (8 - (header_before_padding % 8)) % 8


def _calibration_offset_in_file() -> int:
    return (
        HEADER_SIZE_TEXT
        + HEADER_UID_SIZE
        + HEADER_DEVICE_NAME_LEN
        + HEADER_COMM_SIZE
        + FIRMWARE_VERSION_LEN
        + FREQUENCY_LEN
        + _header_padding_size()
    )


def _read_log_version(data: bytes) -> str:
    header_text_raw = data[:HEADER_SIZE_TEXT]
    header_text = header_text_raw.decode("utf-8", errors="ignore").rstrip("\x00")
    return header_text.replace("FIRM LOG v", "").strip()


def _load_calibration_from_yaml_file(calibration_file: Path) -> dict:
    with calibration_file.open("r", encoding="utf-8") as stream:
        parsed = yaml.safe_load(stream)

    calibration = parsed.get("calibration", parsed)

    return calibration


def _set_calibration(existing_values: list[float], calibration: dict) -> list[float]:
    changed_values = existing_values.copy()

    accel_offset = calibration.get("accel_offset")
    if accel_offset is not None:
        accel_offset = [float(value) for value in accel_offset]
        changed_values[ACCEL_BASE + 0 : ACCEL_BASE + 3] = accel_offset

    accel_matrix = calibration.get("accel_scale")
    if accel_matrix is not None:
        accel_matrix_values: list[float] = []
        for row in accel_matrix:
            accel_matrix_values.extend(float(value) for value in row)
        changed_values[ACCEL_BASE + 3 : ACCEL_BASE + 12] = accel_matrix_values

    gyro_offset = calibration.get("gyro_offset")
    if gyro_offset is not None:
        gyro_offset = [float(value) for value in gyro_offset]
        changed_values[GYRO_BASE + 0 : GYRO_BASE + 3] = gyro_offset

    gyro_matrix = calibration.get("gyro_scale")
    if gyro_matrix is not None:
        gyro_matrix_values: list[float] = []
        for row in gyro_matrix:
            gyro_matrix_values.extend(float(value) for value in row)
        changed_values[GYRO_BASE + 3 : GYRO_BASE + 12] = gyro_matrix_values

    mag_offset = calibration.get("mag_offset")
    if mag_offset is not None:
        mag_offset = [float(value) for value in mag_offset]
        changed_values[MAG_BASE + 0 : MAG_BASE + 3] = mag_offset

    mag_matrix = calibration.get("mag_scale")
    if mag_matrix is not None:
        mag_matrix_values: list[float] = []
        for row in mag_matrix:
            mag_matrix_values.extend(float(value) for value in row)
        changed_values[MAG_BASE + 3 : MAG_BASE + 12] = mag_matrix_values

    high_g_offset = calibration.get("high_g_offset")
    if high_g_offset is not None:
        high_g_offset = [float(value) for value in high_g_offset]
        changed_values[HIGH_G_BASE + 0 : HIGH_G_BASE + 3] = high_g_offset

    high_g_matrix = calibration.get("high_g_scale")
    if high_g_matrix is not None:
        high_g_matrix_values: list[float] = []
        for row in high_g_matrix:
            high_g_matrix_values.extend(float(value) for value in row)
        changed_values[HIGH_G_BASE + 3 : HIGH_G_BASE + 12] = high_g_matrix_values

    return changed_values


def apply_calibration(frm_path: Path, calibration_yaml_path: Path) -> None:
    calibration = _load_calibration_from_yaml_file(calibration_yaml_path)

    raw_bytes = bytearray(frm_path.read_bytes())
    version = _read_log_version(raw_bytes)
    if version != "1.3":
        print("Wrong FIRM log version")
        return
    calibration_offset = _calibration_offset_in_file()
    calibration_end = calibration_offset + CAL_BLOCK_SIZE_BYTES

    calibration_bytes = raw_bytes[calibration_offset:calibration_end]
    existing_values = list(struct.unpack("<" + ("f" * CAL_TOTAL_FLOATS), calibration_bytes))

    changed_values = _set_calibration(existing_values, calibration)
    changed_bytes = struct.pack("<" + ("f" * CAL_TOTAL_FLOATS), *changed_values)

    raw_bytes[calibration_offset:calibration_end] = changed_bytes
    frm_path.write_bytes(raw_bytes)

    print(f"FIRM log version: {version}")
    print(f"Updated calibration in-place: {frm_path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="set_calibration",
        description="Write calibration data in a .frm log using values from a YAML file.",
    )
    parser.add_argument("frm_file", help="Path to the .frm file")
    parser.add_argument("calibration_file", help="Path to YAML file defining calibration values")
    args = parser.parse_args()

    apply_calibration(Path(args.frm_file), Path(args.calibration_file))


if __name__ == "__main__":
    main()
