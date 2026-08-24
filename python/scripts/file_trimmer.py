"""Compatibility wrapper for explicit manager trimming."""

from __future__ import annotations

import argparse
from pathlib import Path

from firm.flight_data.formats import HARDWARE_VERSIONS, LogReader, LogWriter
from firm.flight_data.models import TrimWindow


def trim_file(
    path: str | Path,
    start_seconds: float,
    end_seconds: float,
    *,
    hardware: str,
) -> str:
    source = Path(path).resolve()
    output = source.with_name(f"trimmed_{source.name}")
    reader = LogReader(source)
    LogWriter.migrate(
        source,
        output,
        reader.header.calibration,
        TrimWindow(float(start_seconds), float(end_seconds)),
        source_hardware=hardware,
    )
    return str(output)


def main() -> None:
    parser = argparse.ArgumentParser(description="Trim and migrate a FIRM log")
    parser.add_argument("file", type=Path)
    parser.add_argument("start_seconds", type=float)
    parser.add_argument("end_seconds", type=float)
    parser.add_argument("--hardware", choices=HARDWARE_VERSIONS, required=True)
    args = parser.parse_args()
    print(
        trim_file(
            args.file,
            args.start_seconds,
            args.end_seconds,
            hardware=args.hardware,
        )
    )


if __name__ == "__main__":
    main()
