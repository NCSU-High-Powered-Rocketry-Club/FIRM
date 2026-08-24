"""End-to-end tests for the unified flight-data manager."""

from __future__ import annotations

import hashlib
import shutil
import subprocess
from pathlib import Path
from typing import cast

import firm.flight_data.fingerprint as fingerprint_module
import polars as pl
import pytest
import yaml
from firm.flight_data import Archive, BuildRequest, LogReader, LogWriter, build_recording
from firm.flight_data.calibration import set_calibration_override
from firm.flight_data.cli import main
from firm.flight_data.formats import LATEST_VERSION
from firm.flight_data.trimming import apply_trim_window, propose_phase_trim
from scripts.decoder import decode as compatibility_decode
from scripts.file_trimmer import trim_file as compatibility_trim
from scripts.migrator import new_file as compatibility_migrate
from scripts.set_callibration import apply_calibration as compatibility_calibrate

from integration_tests.flight_log_fixtures import (
    inject_v10_magnetometer_anomaly,
    make_archive,
    write_log,
    write_phase_log,
)


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_ingest_build_override_and_original_immutability(tmp_path: Path) -> None:
    """Build both roles with an override without changing acquisition files."""
    root = make_archive(tmp_path / "flight_data")
    source_flight = write_log(tmp_path / "flight.frm")
    source_cal = write_log(tmp_path / "cal.frm", samples=20)
    archive = Archive(root)
    recording = archive.ingest("launch", "airbrakes", source_flight, source_cal, hardware="new")
    original_hashes = (_digest(recording.flight_original), _digest(recording.mag_cal_original))

    override_path = tmp_path / "override.yaml"
    override_path.write_text("calibration:\n  mag_offset: [1, 2, 3]\n", encoding="utf-8")
    set_calibration_override(recording, override_path)
    result = build_recording(recording, BuildRequest())

    assert result.rebuilt
    assert recording.is_current
    assert original_hashes == (
        _digest(recording.flight_original),
        _digest(recording.mag_cal_original),
    )
    assert LogReader(recording.flight_derived).header.version == LATEST_VERSION
    assert LogReader(recording.flight_derived).header.calibration.values["mag"].offset == (
        1.0,
        2.0,
        3.0,
    )
    assert LogReader(recording.mag_cal_derived).header.calibration.values["mag"].offset == (
        1.0,
        2.0,
        3.0,
    )
    assert recording.flight_derived.stat().st_size < recording.flight_original.stat().st_size
    mag_frame = pl.read_parquet(recording.parquet_path("magnetometer-calibration"))
    assert {"raw_mag_x", "uncalibrated_mag_x_ut", "mag_x_ut"} <= set(mag_frame.columns)
    metadata = pl.read_parquet_metadata(recording.parquet_path("magnetometer"))
    assert metadata["firm.units"]
    assert result.fingerprint in metadata["firm.flight_data"]
    assert all(
        artifact["status"] == "current"
        for artifact in recording.manifest["build"]["artifacts"].values()
    )


@pytest.mark.parametrize("version", ["1.0", "1.1", "1.2", "1.3", "1.4"])
def test_every_historical_version_migrates_directly(tmp_path: Path, version: str) -> None:
    """Every supported version uses the same direct current-version writer."""
    source = write_log(tmp_path / f"v{version}.frm", version=version, samples=6)
    reader = LogReader(source)
    assert reader.validate().valid
    destination = tmp_path / f"v{version}-current.frm"
    hardware = "old" if version in {"1.0", "1.1", "1.2"} else "new"
    count = LogWriter.migrate(
        source, destination, reader.header.calibration, source_hardware=hardware
    )
    assert count == 24
    assert LogReader(destination).header.version == LATEST_VERSION
    assert LogReader(destination).validate().valid


def test_current_magnetometer_packets_are_not_treated_as_v10_anomalies(
    tmp_path: Path,
) -> None:
    """The lazy v1.0 detector sentinel must never alter current-version packets."""
    source = write_log(tmp_path / "current.frm", samples=14)
    payloads = [
        packet.payload
        for packet in LogReader(source).iter_packets()
        if packet.sensor == "magnetometer"
    ]
    assert len(set(payloads)) == 14


def test_three_byte_timestamps_unwrap_across_multiple_cycles(tmp_path: Path) -> None:
    """Historical 24-bit clocks retain a monotonically increasing timeline."""
    source = write_log(tmp_path / "wrapped.frm", version="1.0", samples=40)
    timestamps = [packet.timestamp_s for packet in LogReader(source).iter_packets()]
    assert timestamps == sorted(timestamps)
    assert timestamps[-1] > 0.38


def test_v10_magnetometer_anomaly_correction_is_preserved(tmp_path: Path) -> None:
    """The detected spike cadence is corrected in the normalized event stream."""
    source = write_log(tmp_path / "v1-anomaly.frm", version="1.0", samples=20)
    inject_v10_magnetometer_anomaly(source, 5)
    payloads = [
        packet.payload
        for packet in LogReader(source).iter_packets()
        if packet.sensor == "magnetometer"
    ]
    assert payloads[5] == payloads[4]
    assert payloads[16] == payloads[15]


@pytest.mark.parametrize("tail", [bytes(128), b"\x99garbage", b"I\x01\x02"])
def test_padding_garbage_and_incomplete_tails_stop_at_packet_boundaries(
    tmp_path: Path, tail: bytes
) -> None:
    """Preallocation and interrupted final packets are excluded without losing complete data."""
    source = write_log(tmp_path / "tail.frm", samples=6)
    content = source.read_bytes()[:-128]
    source.write_bytes(content + tail)
    report = LogReader(source).validate()
    destination = tmp_path / "current.frm"
    count = LogWriter.migrate(
        source,
        destination,
        LogReader(source).header.calibration,
        source_hardware="new",
    )

    assert report.valid
    assert report.packet_count == 24
    assert count == 24
    assert LogReader(destination).validate().valid


def test_legacy_scripts_delegate_to_the_shared_format_engine(tmp_path: Path) -> None:
    """Compatibility entry points use the same migration, trim, calibration, and decode code."""
    source = write_log(tmp_path / "historical.frm", version="1.2", samples=20)
    migrated = compatibility_migrate(source, tmp_path / "migrated.frm", hardware="old")
    trimmed = Path(compatibility_trim(source, 0.05, 0.12, hardware="old"))
    calibration = tmp_path / "calibration.yaml"
    calibration.write_text("calibration:\n  mag_offset: [1, 2, 3]\n", encoding="utf-8")
    calibrated = compatibility_calibrate(source, calibration, hardware="old")
    decoded = compatibility_decode(migrated, tmp_path / "decoded")

    assert LogReader(migrated).header.version == LATEST_VERSION
    assert LogReader(trimmed).validate().valid
    assert LogReader(calibrated).header.calibration.values["mag"].offset == (1.0, 2.0, 3.0)
    assert {path.name for path in decoded} == {
        "barometer.parquet",
        "imu.parquet",
        "magnetometer.parquet",
        "high-g.parquet",
    }


def test_explicit_trim_and_stale_fingerprint(tmp_path: Path) -> None:
    """Trim and calibration changes participate in build freshness."""
    root = make_archive(tmp_path / "flight_data")
    recording = Archive(root).ingest(
        "launch",
        "device",
        write_log(tmp_path / "flight.frm", samples=100),
        hardware="new",
    )
    apply_trim_window(recording, start_s=0.2, end_s=0.5)
    built = build_recording(recording)
    report = LogReader(recording.flight_derived).validate()
    assert 100 <= report.packet_count <= 124
    imu = pl.read_parquet(recording.parquet_path("imu"))
    assert cast("float", imu["timestamp_s"].min()) >= 0.2
    assert 0.0 <= cast("float", imu["trimmed_timestamp_s"].min()) < 0.02
    assert recording.manifest["trim_window"]["start_s"] == 0.2
    assert not build_recording(recording).rebuilt

    override = tmp_path / "override.yaml"
    override.write_text("calibration:\n  gyro_offset: [0.1, 0.2, 0.3]\n", encoding="utf-8")
    set_calibration_override(recording, override)
    assert recording.manifest["build"]["status"] == "stale"
    assert build_recording(recording).fingerprint != built.fingerprint


def test_manual_inputs_and_processing_schema_changes_are_stale(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Freshness includes override contents plus decoder and format schema versions."""
    root = make_archive(tmp_path / "flight_data")
    recording = Archive(root).ingest(
        "launch",
        "device",
        write_log(tmp_path / "flight.frm", samples=10),
        hardware="new",
    )
    override = tmp_path / "override.yaml"
    override.write_text("calibration:\n  gyro_offset: [0, 0, 0]\n", encoding="utf-8")
    set_calibration_override(recording, override)
    build_recording(recording)
    assert recording.is_current

    recording.calibration_override.write_text(
        recording.calibration_override.read_text(encoding="utf-8") + "# edited\n",
        encoding="utf-8",
    )
    assert not recording.is_current
    set_calibration_override(recording, override)
    build_recording(recording)
    decoder_version = fingerprint_module.DECODER_SCHEMA_VERSION
    monkeypatch.setattr(
        fingerprint_module,
        "DECODER_SCHEMA_VERSION",
        decoder_version + 1,
    )
    assert not recording.is_current
    monkeypatch.setattr(fingerprint_module, "DECODER_SCHEMA_VERSION", decoder_version)
    assert recording.is_current
    monkeypatch.setattr(
        fingerprint_module,
        "FORMAT_SCHEMA_VERSION",
        fingerprint_module.FORMAT_SCHEMA_VERSION + 1,
    )
    assert not recording.is_current


def test_changed_immutable_source_is_rejected(tmp_path: Path) -> None:
    """A changed acquisition file cannot remain current or be rebuilt."""
    root = make_archive(tmp_path / "flight_data")
    recording = Archive(root).ingest(
        "launch",
        "device",
        write_log(tmp_path / "flight.frm", samples=10),
        hardware="new",
    )
    build_recording(recording)
    with recording.flight_original.open("ab") as handle:
        handle.write(b"changed")

    assert not recording.is_current
    with pytest.raises(ValueError, match="immutable original changed"):
        build_recording(recording)


def test_phase_detection_requires_confident_flight(tmp_path: Path) -> None:
    """Static data cannot produce an accepted-looking phase proposal."""
    root = make_archive(tmp_path / "flight_data")
    recording = Archive(root).ingest(
        "launch",
        "static",
        write_log(tmp_path / "static.frm", samples=40),
        hardware="new",
    )
    with pytest.raises(ValueError, match="liftoff"):
        propose_phase_trim(recording)


def test_phase_proposal_requires_acceptance_and_applies_padding(tmp_path: Path) -> None:
    """A corroborated proposal remains reviewable until explicitly accepted."""
    root = make_archive(tmp_path / "flight_data")
    recording = Archive(root).ingest(
        "launch",
        "flight",
        write_phase_log(tmp_path / "phases.frm"),
        hardware="new",
    )
    proposal = propose_phase_trim(recording, before_s=0.2, after_s=0.3)

    assert proposal.confidence >= 0.8
    assert 0.79 <= proposal.start_s <= 0.81
    assert recording.manifest["trim_window"] is None
    accepted = apply_trim_window(recording, proposal_id=proposal.proposal_id)
    assert accepted.source == "phase_detection"
    assert accepted.proposal_id == proposal.proposal_id
    build_recording(recording)
    timestamps = pl.read_parquet(recording.parquet_path("imu"))["timestamp_s"]
    assert cast("float", timestamps.min()) >= proposal.start_s
    assert cast("float", timestamps.max()) <= proposal.end_s


def test_empty_real_shape_cli_lists_no_data(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """An empty catalog has a successful but empty list command."""
    root = make_archive(tmp_path / "flight_data")
    assert main(["--archive", str(root), "list"]) == 0
    assert "No managed recordings" in capsys.readouterr().out
    assert yaml.safe_load((root / "archive.yaml").read_text())["schema_version"] == 1


def test_repository_archive_is_empty_without_touching_legacy_datasets(
    capsys: pytest.CaptureFixture[str],
) -> None:
    """The committed catalog is empty and listing it leaves legacy data byte-identical."""
    repository = Path(__file__).resolve().parents[1]
    legacy_root = repository / "eskf_lab" / "datasets"
    before = {
        path.relative_to(legacy_root): _digest(path)
        for path in legacy_root.rglob("*")
        if path.is_file()
    }
    assert Archive(repository / "flight_data").recordings() == []
    assert main(["--archive", str(repository / "flight_data"), "list"]) == 0
    assert "No managed recordings" in capsys.readouterr().out
    after = {
        path.relative_to(legacy_root): _digest(path)
        for path in legacy_root.rglob("*")
        if path.is_file()
    }
    assert after == before


@pytest.mark.integration
def test_manager_output_is_accepted_by_rust_playback(tmp_path: Path) -> None:
    """Feed a manager-generated current log through the Rust playback parser."""
    cargo = shutil.which("cargo")
    if cargo is None:
        pytest.skip("Cargo is required for the Rust playback compatibility test")
    root = make_archive(tmp_path / "flight_data")
    recording = Archive(root).ingest(
        "launch",
        "device",
        write_log(tmp_path / "flight.frm", samples=8),
        hardware="new",
    )
    build_recording(recording)
    repository = Path(__file__).resolve().parents[1]
    completed = subprocess.run(  # noqa: S603 -- Cargo executable is resolved from PATH.
        [
            cargo,
            "run",
            "--offline",
            "-q",
            "-p",
            "firm_rust",
            "--example",
            "verify_mock_packets",
            "--",
            str(recording.flight_derived),
        ],
        cwd=repository / "FIRM-Client",
        check=True,
        capture_output=True,
        text=True,
    )
    assert "OK: total=32 B=8 I=8 M=8 A=8" in completed.stdout
