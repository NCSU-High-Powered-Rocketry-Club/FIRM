"""Compatibility wrapper for the unified FIRM log decoder."""

from __future__ import annotations

import argparse
from pathlib import Path

from firm.flight_data.decoding import decode_to_parquet
from firm.flight_data.formats import LogReader


def decode(path: str | Path, output: str | Path = ".") -> list[Path]:
    source = Path(path).resolve()
    reader = LogReader(source)
    written, _counts = decode_to_parquet(
        source,
        Path(output).resolve(),
        reader.header.calibration,
        metadata={"source": str(source), "compatibility_wrapper": "decoder.py"},
    )
    return list(written.values())


def main() -> None:
    parser = argparse.ArgumentParser(description="Decode a FIRM log into native-rate Parquet")
    parser.add_argument("file", type=Path)
    parser.add_argument("--output", type=Path, default=Path.cwd())
    args = parser.parse_args()
    for path in decode(args.file, args.output):
        print(path)


if __name__ == "__main__":
    main()
