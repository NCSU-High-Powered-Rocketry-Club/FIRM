"""Compatibility wrapper for direct migration through the unified format registry."""

from __future__ import annotations

import argparse
from pathlib import Path

from firm.flight_data.formats import HARDWARE_VERSIONS, LogReader, LogWriter


def new_file(path: str | Path, output: str | Path = "migrated_log.FRM", *, hardware: str) -> Path:
    source = Path(path).resolve()
    destination = Path(output).resolve()
    reader = LogReader(source)
    LogWriter.migrate(source, destination, reader.header.calibration, source_hardware=hardware)
    return destination


def main() -> None:
    parser = argparse.ArgumentParser(description="Migrate a FIRM log to the current format")
    parser.add_argument("file", type=Path)
    parser.add_argument("--output", type=Path, default=Path("migrated_log.FRM"))
    parser.add_argument("--hardware", choices=HARDWARE_VERSIONS, required=True)
    args = parser.parse_args()
    print(new_file(args.file, args.output, hardware=args.hardware))


if __name__ == "__main__":
    main()
