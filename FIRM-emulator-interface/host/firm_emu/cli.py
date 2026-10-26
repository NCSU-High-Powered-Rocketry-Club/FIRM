"""Command line entry point.

firm-emu inspect LOG
firm-emu compare EXPECTED ACTUAL [--tolerance-ms 1.0] [--seconds N]
firm-emu run LOG --out OUT [--speed 1.0] [--seconds N]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from firm_emu.log import by_sensor, compare_logs, first_seconds, read_log, sensor_stats
from firm_emu.replay import FileOutput, replay


def inspect(path: Path) -> int:
    """Print the header and per-sensor timing of a log."""
    header, records = read_log(path)
    duration = records[-1].time_s if records else 0.0
    print(f"{path.name}: log v{header.version}, firmware {header.firmware_version}")
    print(f"{len(records)} records over {duration:.1f} s")
    print()
    print(f"{'sensor':<14}{'records':>9}{'rate':>10}{'median':>12}{'max gap':>12}")
    for name, group in by_sensor(records).items():
        s = sensor_stats(group)
        print(
            f"{name:<14}{s.count:>9}{s.rate_hz:>7.0f} Hz"
            f"{s.median_interval_ms:>9.2f} ms{s.max_gap_ms:>9.1f} ms"
        )
    return 0


def compare(expected: Path, actual: Path, tolerance_ms: float, seconds: float | None) -> int:
    """Print how FIRM's log lines up with the replayed log. Returns 1 on failure."""
    diffs = compare_logs(expected, actual, tolerance_ms, seconds)
    print(
        f"{'sensor':<14}{'matched':>9}{'late':>7}{'missing':>9}{'extra':>7}{'wrong':>7}"
        f"{'worst':>11}{'drift':>11}"
    )
    for name, d in diffs.items():
        print(
            f"{name:<14}{d.matched:>9}{d.late:>7}{d.missing:>9}{d.extra:>7}{d.wrong:>7}"
            f"{d.worst_ms:>8.3f} ms{d.drift_ms:>8.3f} ms"
        )
    print()
    failed = {name: d for name, d in diffs.items() if not d.passed}
    for name, d in failed.items():
        if d.first_problem_s is None:
            print(f"{name}: too many missing records")
        else:
            print(f"{name}: first problem at {d.first_problem_s:.3f} s")
    print("FAIL" if failed else "PASS")
    return 1 if failed else 0


def run(log: Path, out: Path, speed: float, seconds: float | None) -> int:
    """Replay a log in real time into a file."""
    _, records = read_log(log)
    records = first_seconds(records, seconds)
    print(f"replaying {len(records)} records at {speed}x ...")
    stats = replay(records, FileOutput(out), speed)
    print(f"sent {stats.sent} records in {stats.duration_s:.1f} s")
    print(f"worst delay {stats.max_late_ms:.3f} ms")
    print(f"wrote {out}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="firm-emu", description="FIRM emulator tools")
    commands = parser.add_subparsers(dest="command", required=True)

    inspect_cmd = commands.add_parser("inspect", help="summarize a .frm log")
    inspect_cmd.add_argument("log", type=Path)

    compare_cmd = commands.add_parser("compare", help="check FIRM's log against a replayed log")
    compare_cmd.add_argument("expected", type=Path, help="the log that was replayed")
    compare_cmd.add_argument("actual", type=Path, help="the log FIRM wrote")
    compare_cmd.add_argument("--tolerance-ms", type=float, default=1.0)
    compare_cmd.add_argument("--seconds", type=float, help="only compare the first N seconds")

    run_cmd = commands.add_parser("run", help="replay a log at its original timing")
    run_cmd.add_argument("log", type=Path)
    run_cmd.add_argument("--out", type=Path, required=True, help="write the replay to this .frm")
    run_cmd.add_argument("--speed", type=float, default=1.0, help="2 replays twice as fast")
    run_cmd.add_argument("--seconds", type=float, help="only replay the first N seconds")

    args = parser.parse_args(argv)
    if args.command == "inspect":
        return inspect(args.log)
    if args.command == "compare":
        return compare(args.expected, args.actual, args.tolerance_ms, args.seconds)
    return run(args.log, args.out, args.speed, args.seconds)


if __name__ == "__main__":
    sys.exit(main())
