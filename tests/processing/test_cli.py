"""Command-line result discovery tests."""

from __future__ import annotations

import json
from typing import TYPE_CHECKING

from firm.eskf_lab.cli import _latest_results, _parser, main

from .flight_data_fixture import make_archive

if TYPE_CHECKING:
    from pathlib import Path

    import pytest


def test_latest_results_discovers_each_dataset_once(tmp_path: Path) -> None:
    """The all-dataset dashboard selects only each dataset's declared latest run."""
    results = tmp_path / "results"
    for dataset in ("launch-bravo", "launch-alpha"):
        run = results / dataset / "20260813T010203Z-hash"
        run.mkdir(parents=True)
        (run / "result.parquet").touch()
        (run.parent / "latest.json").write_text(
            json.dumps({"result_directory": str(run)}), encoding="utf-8"
        )
    (results / "comparison-only").mkdir()

    discovered = _latest_results(results)

    assert list(discovered) == ["launch-alpha", "launch-bravo"]
    assert all(path.name == "result.parquet" for path in discovered.values())


def test_run_force_bypasses_native_tests() -> None:
    """The explicit force flag maps to the native-test bypass."""
    args = _parser().parse_args(["run", "launch-alpha", "--force"])

    assert args.skip_native_tests is True
    assert args.force_prepare is False


def test_legacy_skip_native_tests_alias_is_retained() -> None:
    """Existing scripts using the descriptive flag continue to work."""
    args = _parser().parse_args(["run", "launch-alpha", "--skip-native-tests"])

    assert args.skip_native_tests is True


def test_empty_manager_cli_contract(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """Empty list succeeds while data-consuming commands explain the manager workflow."""
    archive = make_archive(tmp_path / "flight_data")
    common = ["--flight-data-dir", str(archive)]

    assert main([*common, "list"]) == 0
    assert "No current managed recordings" in capsys.readouterr().out
    assert main([*common, "prepare"]) == 2
    assert "ingest and build data with 'firm-log'" in capsys.readouterr().err
    assert main([*common, "run"]) == 2
    assert "ingest and build data with 'firm-log'" in capsys.readouterr().err
    assert main([*common, "serve"]) == 2
    assert "ingest and build data with 'firm-log'" in capsys.readouterr().err
