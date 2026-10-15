"""Command-line interface for the unified FIRM flight-data manager."""

from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence
from pathlib import Path

import yaml

from .archive import Archive, sha256_file
from .build import build_fingerprint, build_recording
from .calibration import effective_calibration, set_calibration_override
from .decoding import export_csv
from .formats import HARDWARE_VERSIONS, TARGET_HARDWARE, LogReader
from .models import BuildRequest, BuildResult
from .trimming import apply_trim_window, propose_phase_trim


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="firm-log", description="Manage versioned FIRM logs")
    parser.add_argument("--archive", type=Path, help="flight_data archive root")
    commands = parser.add_subparsers(dest="command", required=True)

    ingest = commands.add_parser("ingest", help="copy immutable acquisition logs into the archive")
    ingest.add_argument("--launch", required=True)
    ingest.add_argument("--recording", required=True)
    ingest.add_argument("--flight", type=Path, required=True)
    ingest.add_argument("--mag-cal", type=Path)
    ingest.add_argument("--hardware", choices=HARDWARE_VERSIONS, required=True)
    ingest.set_defaults(handler=_ingest)

    list_command = commands.add_parser("list", help="list managed recordings")
    list_command.set_defaults(handler=_list)

    inspect = commands.add_parser("inspect", help="inspect a recording and its log header")
    inspect.add_argument("recording")
    source = inspect.add_mutually_exclusive_group()
    source.add_argument("--original", action="store_true")
    source.add_argument("--derived", action="store_true")
    inspect.set_defaults(handler=_inspect)

    validate = commands.add_parser("validate", help="validate recording originals")
    validate.add_argument("recording")
    validate.set_defaults(handler=_validate)

    status = commands.add_parser("status", help="show current, stale, or unbuilt state")
    status.add_argument("recordings", nargs="*")
    status.set_defaults(handler=_status)

    calibration = commands.add_parser("calibration", help="show or set a recording override")
    calibration_commands = calibration.add_subparsers(dest="calibration_command", required=True)
    calibration_show = calibration_commands.add_parser("show")
    calibration_show.add_argument("recording")
    calibration_show.set_defaults(handler=_calibration_show)
    calibration_set = calibration_commands.add_parser("set")
    calibration_set.add_argument("recording")
    calibration_set.add_argument("file", type=Path)
    calibration_set.set_defaults(handler=_calibration_set)

    hardware = commands.add_parser("hardware", help="show or set source hardware generation")
    hardware_commands = hardware.add_subparsers(dest="hardware_command", required=True)
    hardware_show = hardware_commands.add_parser("show")
    hardware_show.add_argument("recording")
    hardware_show.set_defaults(handler=_hardware_show)
    hardware_set = hardware_commands.add_parser("set")
    hardware_set.add_argument("recording")
    hardware_set.add_argument("hardware", choices=HARDWARE_VERSIONS)
    hardware_set.set_defaults(handler=_hardware_set)

    trim = commands.add_parser("trim", help="preview or accept a flight trim window")
    trim_commands = trim.add_subparsers(dest="trim_command", required=True)
    preview = trim_commands.add_parser("preview")
    preview.add_argument("recording")
    preview.add_argument("--phases", action="store_true", required=True)
    preview.add_argument("--before", type=float, default=5.0)
    preview.add_argument("--after", type=float, default=10.0)
    preview.set_defaults(handler=_trim_preview)
    apply = trim_commands.add_parser("apply")
    apply.add_argument("recording")
    apply.add_argument("--proposal")
    apply.add_argument("--start", type=float)
    apply.add_argument("--end", type=float)
    apply.set_defaults(handler=_trim_apply)

    build = commands.add_parser("build", help="migrate and decode one recording")
    build.add_argument("recording")
    build.add_argument("--target", default="latest")
    build.add_argument("--force", action="store_true")
    build.set_defaults(handler=_build)

    rebuild = commands.add_parser("rebuild", help="build every managed recording")
    rebuild.add_argument("--all", action="store_true", required=True)
    rebuild.add_argument("--target", default="latest")
    rebuild.add_argument("--force", action="store_true")
    rebuild.set_defaults(handler=_rebuild)

    export = commands.add_parser("export", help="export analysis artifacts")
    export_commands = export.add_subparsers(dest="export_command", required=True)
    export_csv_command = export_commands.add_parser("csv")
    export_csv_command.add_argument("recording")
    export_csv_command.add_argument("--output", type=Path, required=True)
    export_csv_command.set_defaults(handler=_export_csv)
    return parser


def _archive(args: argparse.Namespace) -> Archive:
    return Archive(args.archive)


def _ingest(args: argparse.Namespace) -> int:
    recording = _archive(args).ingest(
        args.launch,
        args.recording,
        args.flight,
        args.mag_cal,
        hardware=args.hardware,
    )
    print(f"Ingested {recording.dataset_id}")
    print(f"  {recording.flight_original}")
    if recording.mag_cal_original.is_file():
        print(f"  {recording.mag_cal_original}")
    return 0


def _list(args: argparse.Namespace) -> int:
    recordings = _archive(args).recordings()
    if not recordings:
        print("No managed recordings are available. Use 'firm-log ingest' to add one.")
        return 0
    for recording in recordings:
        status = recording.manifest.get("build", {}).get("status", "unknown")
        print(f"{recording.dataset_id:<48} {status}")
    return 0


def _header_mapping(path: Path) -> dict[str, object]:
    header = LogReader(path).header
    return {
        "path": str(path),
        "log_version": header.version,
        "device_uid": header.device_uid,
        "device_name": header.device_name,
        "communications": list(header.communications),
        "firmware_version": header.firmware_version,
        "frequency_hz": header.frequency_hz,
        "calibration": header.calibration.to_mapping()["calibration"],
    }


def _inspect(args: argparse.Namespace) -> int:
    recording = _archive(args).resolve(args.recording)
    document: dict[str, object] = {"manifest": recording.manifest}
    use_derived = args.derived
    flight = recording.flight_derived if use_derived else recording.flight_original
    document["flight_header"] = _header_mapping(flight)
    mag = recording.mag_cal_derived if use_derived else recording.mag_cal_original
    if mag.is_file():
        document["magnetometer_calibration_header"] = _header_mapping(mag)
    print(yaml.safe_dump(document, sort_keys=False).rstrip())
    return 0


def _validate(args: argparse.Namespace) -> int:
    recording = _archive(args).resolve(args.recording)
    reports = {"flight": LogReader(recording.flight_original).validate().to_mapping()}
    if recording.mag_cal_original.is_file():
        reports["magnetometer_calibration"] = (
            LogReader(recording.mag_cal_original).validate().to_mapping()
        )
    source_paths = {
        "flight": recording.flight_original,
        "magnetometer_calibration": recording.mag_cal_original,
    }
    for role, source in recording.manifest.get("sources", {}).items():
        report = reports.get(role)
        path = source_paths.get(role)
        if report is None or path is None:
            continue
        expected_hash = source.get("sha256") if isinstance(source, dict) else None
        if not path.is_file() or sha256_file(path) != expected_hash:
            report["valid"] = False
            report.setdefault("errors", []).append(
                "immutable source SHA-256 does not match manifest"
            )
    print(yaml.safe_dump(reports, sort_keys=False).rstrip())
    return 0 if all(bool(report["valid"]) for report in reports.values()) else 2


def _status(args: argparse.Namespace) -> int:
    archive = _archive(args)
    recordings = (
        [archive.resolve(value) for value in args.recordings]
        if args.recordings
        else archive.recordings()
    )
    if not recordings:
        print("No managed recordings are available. Use 'firm-log ingest' to add one.")
        return 0
    for recording in recordings:
        build = recording.manifest.get("build", {})
        expected = build_fingerprint(recording)
        if recording.is_current and build.get("fingerprint") == expected:
            state = "current"
        elif build.get("status") == "not_built":
            state = "not_built"
        else:
            state = "stale"
        print(f"{recording.dataset_id:<48} {state}")
    return 0


def _calibration_show(args: argparse.Namespace) -> int:
    recording = _archive(args).resolve(args.recording)
    print(yaml.safe_dump(effective_calibration(recording).to_mapping(), sort_keys=False).rstrip())
    return 0


def _calibration_set(args: argparse.Namespace) -> int:
    recording = _archive(args).resolve(args.recording)
    set_calibration_override(recording, args.file)
    print(f"Updated calibration override for {recording.dataset_id}; build is stale")
    return 0


def _hardware_show(args: argparse.Namespace) -> int:
    recording = _archive(args).resolve(args.recording)
    print(f"source: {recording.source_hardware}")
    print(f"migration target: {TARGET_HARDWARE}")
    return 0


def _hardware_set(args: argparse.Namespace) -> int:
    recording = _archive(args).resolve(args.recording)
    recording.set_source_hardware(args.hardware)
    print(f"Set source hardware for {recording.dataset_id} to {args.hardware}; build is stale")
    return 0


def _trim_preview(args: argparse.Namespace) -> int:
    recording = _archive(args).resolve(args.recording)
    proposal = propose_phase_trim(recording, before_s=args.before, after_s=args.after)
    print(yaml.safe_dump(proposal.to_mapping(), sort_keys=False).rstrip())
    print(
        f"Accept with: firm-log trim apply {recording.dataset_id} --proposal {proposal.proposal_id}"
    )
    return 0


def _trim_apply(args: argparse.Namespace) -> int:
    recording = _archive(args).resolve(args.recording)
    if args.proposal and (args.start is not None or args.end is not None):
        raise ValueError("choose a proposal or an explicit start/end interval, not both")
    window = apply_trim_window(
        recording, start_s=args.start, end_s=args.end, proposal_id=args.proposal
    )
    print(yaml.safe_dump(window.to_mapping(), sort_keys=False).rstrip())
    return 0


def _print_build(result: object) -> None:
    if not isinstance(result, BuildResult):
        raise TypeError("expected a BuildResult")
    action = "Built" if result.rebuilt else "Already current"
    print(f"{action}: {result.recording_id} ({result.fingerprint[:12]})")
    for name, path in result.artifacts.items():
        print(f"  {name}: {path}")


def _build(args: argparse.Namespace) -> int:
    recording = _archive(args).resolve(args.recording)
    result = build_recording(recording, BuildRequest(args.target, args.force))
    _print_build(result)
    return 0


def _rebuild(args: argparse.Namespace) -> int:
    recordings = _archive(args).recordings()
    if not recordings:
        raise FileNotFoundError("no managed recordings; ingest data with firm-log first")
    for recording in recordings:
        _print_build(build_recording(recording, BuildRequest(args.target, args.force)))
    return 0


def _export_csv(args: argparse.Namespace) -> int:
    recording = _archive(args).resolve(args.recording)
    for path in export_csv(recording.decoded_dir, args.output):
        print(path)
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        return int(args.handler(args))
    except (FileExistsError, FileNotFoundError, OSError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
