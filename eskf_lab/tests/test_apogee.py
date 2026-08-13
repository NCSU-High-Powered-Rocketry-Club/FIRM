"""Optional HPRM apogee-prediction tests."""

from __future__ import annotations

import json
import math
from typing import TYPE_CHECKING

import polars as pl
import pytest
from firm.eskf_lab.apogee import (
    HPRM_APOGEE_COLUMN,
    add_hprm_apogee_predictions,
    load_rocket_properties,
)
from firm.eskf_lab.visualize import make_figure, select_default_columns

if TYPE_CHECKING:
    from pathlib import Path


def _write_properties(dataset: Path) -> Path:
    path = dataset / "rocket_properties.json"
    path.write_text(
        json.dumps(
            {
                "rocket": {
                    "rocket_Cd": 0.393,
                    "rocket_dry_mass_kg": 16.601,
                    "rocket_cross_sectional_area_m2": 0.0182414692475,
                }
            }
        ),
        encoding="utf-8",
    )
    return path


def test_dataset_without_properties_omits_prediction(tmp_path: Path) -> None:
    """A dataset without the optional file keeps its original result schema."""
    frame = pl.DataFrame(
        {
            "timestamp": [0.0],
            "eskf_position_z_m": [0.0],
            "eskf_velocity_z_mps": [0.0],
        }
    )

    result, summary = add_hprm_apogee_predictions(frame, tmp_path)

    assert summary is None
    assert HPRM_APOGEE_COLUMN not in result.columns
    assert load_rocket_properties(tmp_path) is None


def test_hprm_prediction_uses_eskf_coast_state(tmp_path: Path) -> None:
    """Predictions begin after peak ESKF velocity and end before descent."""
    _write_properties(tmp_path)
    frame = pl.DataFrame(
        {
            "timestamp": [0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
            "eskf_position_z_m": [0.0, 25.0, 100.0, 150.0, 180.0, 181.0],
            "eskf_velocity_z_mps": [0.0, 20.0, 50.0, 40.0, 10.0, -1.0],
        }
    )

    result, summary = add_hprm_apogee_predictions(frame, tmp_path)
    predictions = result[HPRM_APOGEE_COLUMN]

    assert summary is not None
    assert summary.prediction_rows == 2
    assert summary.first_prediction_timestamp_seconds == 3.0
    assert summary.last_prediction_timestamp_seconds == 4.0
    assert predictions[:3].is_nan().all()
    assert predictions[3] > frame["eskf_position_z_m"][3]
    assert predictions[4] > frame["eskf_position_z_m"][4]
    assert math.isnan(predictions[5])


def test_invalid_properties_are_reported(tmp_path: Path) -> None:
    """An existing but invalid properties file is not silently ignored."""
    (tmp_path / "rocket_properties.json").write_text(
        json.dumps(
            {
                "rocket": {
                    "rocket_Cd": 0.3,
                    "rocket_dry_mass_kg": -1.0,
                    "rocket_cross_sectional_area_m2": 0.01,
                }
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match=r"rocket_dry_mass_kg.*positive"):
        load_rocket_properties(tmp_path)


def test_prediction_is_a_default_plot_column(tmp_path: Path) -> None:
    """Prediction-enabled results show the HPRM plot without CLI flags."""
    path = tmp_path / "result.parquet"
    pl.DataFrame(
        {
            "timestamp": [0.0],
            "eskf_position_z_m": [0.0],
            "hprm_predicted_apogee_m": [100.0],
        }
    ).write_parquet(path)

    columns = select_default_columns(path)
    figure = make_figure([path], columns)

    assert HPRM_APOGEE_COLUMN in columns
    assert "HPRM predicted apogee (m)" in [
        annotation.text for annotation in figure.layout.annotations
    ]
