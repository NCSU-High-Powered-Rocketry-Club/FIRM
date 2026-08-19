"""Command-line result discovery tests."""

from __future__ import annotations

import json
from typing import TYPE_CHECKING

from firm.eskf_lab.cli import _latest_results, _parser

if TYPE_CHECKING:
    from pathlib import Path


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
