"""Manager-backed ESKF dataset discovery and cache tests."""

from __future__ import annotations

from pathlib import Path

import polars as pl
from firm.eskf_lab.dataset import discover_datasets, prepare_dataset, resolve_dataset
from firm.eskf_lab.profile import load_profile
from firm.flight_data import Archive, build_recording
from firm.flight_data.calibration import set_calibration_override

from .flight_data_fixture import make_archive, write_log


def _profile() -> object:
    return load_profile(Path(__file__).parents[1] / "config" / "default.toml")


def _built_recording(tmp_path: Path):
    archive_root = make_archive(tmp_path / "flight_data")
    flight = write_log(tmp_path / "flight.frm", samples=400)
    recording = Archive(archive_root).ingest("jackpot-4", "airbrakes", flight, hardware="new")
    build_recording(recording)
    return archive_root, recording


def test_discovery_uses_only_current_manager_recordings(tmp_path: Path) -> None:
    """Unbuilt recordings are not exposed to ESKF discovery."""
    archive_root = make_archive(tmp_path / "flight_data")
    flight = write_log(tmp_path / "flight.frm", samples=20)
    unbuilt = Archive(archive_root).ingest("launch", "unbuilt", flight, hardware="new")
    assert discover_datasets(archive_root, _profile()) == []
    build_recording(unbuilt)
    assert [item.dataset_id for item in discover_datasets(archive_root, _profile())] == [
        "launch/unbuilt"
    ]
    assert resolve_dataset("launch/unbuilt", archive_root, _profile()).dataset_id == (
        "launch/unbuilt"
    )


def test_prepare_reads_manager_parquet_and_reuses_cache(tmp_path: Path) -> None:
    """ESKF aligns manager Parquet and reuses a fingerprinted cache."""
    _archive_root, recording = _built_recording(tmp_path)
    profile = _profile()
    first = prepare_dataset(recording, profile, tmp_path / "cache")
    second = prepare_dataset(recording, profile, tmp_path / "cache")
    frame = pl.read_parquet(first.parquet_path)

    assert first.dataset_id == "jackpot-4/airbrakes"
    assert first.cache_path == second.cache_path
    assert first.metadata["fingerprint"] == second.metadata["fingerprint"]
    assert first.metadata["manager_build_fingerprint"] == recording.manifest["build"]["fingerprint"]
    assert {"imu_accel_z_g", "pressure_pa", "mag_x_ut"} <= set(frame.columns)
    assert "imu__raw_accel_z" in frame.columns


def test_calibration_override_rebuild_changes_eskf_fingerprint(tmp_path: Path) -> None:
    """A rebuilt calibration produces a new ESKF input fingerprint."""
    _archive_root, recording = _built_recording(tmp_path)
    profile = _profile()
    first = prepare_dataset(recording, profile, tmp_path / "cache")
    override = tmp_path / "override.yaml"
    override.write_text("calibration:\n  accel_offset: [0, 0, 0.5]\n", encoding="utf-8")
    set_calibration_override(recording, override)
    build_recording(recording)
    second = prepare_dataset(recording, profile, tmp_path / "cache")
    frame = pl.read_parquet(second.parquet_path)

    assert first.metadata["fingerprint"] != second.metadata["fingerprint"]
    assert frame["imu_accel_z_g"][0] == 0.5
