"""Dataset discovery and cache tests."""

from __future__ import annotations

from pathlib import Path

import polars as pl
from firm.eskf_lab.dataset import discover_datasets, prepare_dataset
from firm.eskf_lab.profile import load_profile

PREAMBLE = """Test FIRM,123
FIRM version:,v2.0.0
usb enabled:,True
uart enabled:,False
i2c enabled:,False
spi enabled:,False
Transmit Frequency:,100
ICM45686 Acceleration Calibration,0
ICM45686 Gyroscope Calibration,0
MMC5983MA Magnetometer Calibration,0
ADXL371 Acceleration Calibration,0

"""


def _write(path: Path, header: str, rows: list[str]) -> None:
    path.write_text(PREAMBLE + header + "\n" + "\n".join(rows) + "\n", encoding="utf-8")


def test_prepare_uses_top_level_sensor_csvs_and_keeps_unknown_columns(tmp_path: Path) -> None:
    """Preparation ignores Calibration and retains newly introduced sensor columns."""
    dataset = tmp_path / "launch"
    dataset.mkdir()
    calibration = dataset / "Calibration"
    calibration.mkdir()
    timestamps = [f"{index * 0.01:.2f}" for index in range(400)]
    _write(
        dataset / "ICM45686_data.csv",
        "timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,new_signal",
        [f"{time},0,0,1,0,0,0,{index}" for index, time in enumerate(timestamps)],
    )
    _write(
        dataset / "BMP581_data.csv",
        "timestamp,temperature,pressure",
        [f"{time},20,101325" for time in timestamps],
    )
    _write(
        dataset / "MMC5983MA_data.csv",
        "timestamp,mag_x,mag_y,mag_z",
        [f"{time},20,5,40" for time in timestamps],
    )
    _write(
        dataset / "ADXL371_data.csv",
        "timestamp,accel_x,accel_y,accel_z",
        [f"{time},0,0,1" for time in timestamps],
    )
    _write(
        calibration / "MMC5983MA_data.csv",
        "timestamp,mag_x,mag_y,mag_z",
        ["0.0,999,999,999"],
    )

    profile = load_profile(Path(__file__).parents[1] / "config" / "default.toml")
    assert discover_datasets(tmp_path, profile) == [dataset]
    prepared = prepare_dataset(dataset, profile, tmp_path / "cache")
    frame = pl.read_parquet(prepared.parquet_path)

    assert frame.height == 400
    assert "imu__new_signal" in frame.columns
    assert frame["mag_x_ut"].max() == 20
    assert prepared.replay_path.stat().st_size > 40


def test_prepare_cache_is_reused(tmp_path: Path) -> None:
    """An unchanged source dataset resolves to its existing content-addressed cache."""
    dataset = tmp_path / "launch"
    dataset.mkdir()
    timestamps = ["0.0", "1.0", "2.0", "3.0"]
    _write(
        dataset / "ICM45686_data.csv",
        "timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z",
        [f"{time},0,0,1,0,0,0" for time in timestamps],
    )
    _write(
        dataset / "BMP581_data.csv",
        "timestamp,temperature,pressure",
        [f"{time},20,101325" for time in timestamps],
    )
    _write(
        dataset / "MMC5983MA_data.csv",
        "timestamp,mag_x,mag_y,mag_z",
        [f"{time},20,5,40" for time in timestamps],
    )
    _write(
        dataset / "ADXL371_data.csv",
        "timestamp,accel_x,accel_y,accel_z",
        [f"{time},0,0,1" for time in timestamps],
    )
    profile = load_profile(Path(__file__).parents[1] / "config" / "default.toml")
    first = prepare_dataset(dataset, profile, tmp_path / "cache")
    second = prepare_dataset(dataset, profile, tmp_path / "cache")
    assert first.cache_path == second.cache_path
    assert first.metadata["fingerprint"] == second.metadata["fingerprint"]
