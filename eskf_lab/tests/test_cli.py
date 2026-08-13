"""Command-line result discovery tests."""

from __future__ import annotations

import json
from typing import TYPE_CHECKING

from firm.eskf_lab.cli import _latest_results

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
