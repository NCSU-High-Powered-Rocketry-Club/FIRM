"""Compatibility wrapper for calibration-overlay migration.

The historical script modified acquisition logs in place. This wrapper deliberately creates a
new current-version file so originals remain immutable. New workflows should use `firm-log
calibration set` followed by `firm-log build`.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml

from firm.flight_data.formats import HARDWARE_VERSIONS, LogReader, LogWriter
from firm.flight_data.models import CalibrationOverride


def apply_calibration(frm_path: Path, calibration_yaml_path: Path, *, hardware: str) -> Path:
    document = yaml.safe_load(calibration_yaml_path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("calibration override must be a YAML mapping")
    reader = LogReader(frm_path)
    calibration = CalibrationOverride.from_mapping(document).apply(reader.header.calibration)
    destination = frm_path.with_name(f"calibrated_{frm_path.name}")
    LogWriter.migrate(frm_path, destination, calibration, source_hardware=hardware)
    return destination


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="set_calibration",
        description="Create a calibrated current-version copy of a .frm log",
    )
    parser.add_argument("frm_file", type=Path)
    parser.add_argument("calibration_file", type=Path)
    parser.add_argument("--hardware", choices=HARDWARE_VERSIONS, required=True)
    args = parser.parse_args()
    print(
        apply_calibration(
            args.frm_file.resolve(),
            args.calibration_file.resolve(),
            hardware=args.hardware,
        )
    )


if __name__ == "__main__":
    main()
