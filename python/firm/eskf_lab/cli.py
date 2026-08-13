"""Command-line interface for preparing, replaying, and inspecting ESKF runs."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import TYPE_CHECKING

import polars as pl

from .dataset import (
    PreparedDataset,
    discover_datasets,
    human_size,
    prepare_dataset,
    resolve_dataset,
)
from .native import build_native, run_replay
from .paths import (
    DEFAULT_BUILD_DIR,
    DEFAULT_CACHE_DIR,
    DEFAULT_DATASETS_DIR,
    DEFAULT_PROFILE,
    DEFAULT_RESULTS_DIR,
)
from .profile import DatasetProfile, load_profile
from .visualize import result_columns, select_default_columns, serve_results, write_report

if TYPE_CHECKING:
    from collections.abc import Sequence


def _add_common_paths(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--datasets-dir", type=Path, default=DEFAULT_DATASETS_DIR)
    parser.add_argument("--cache-dir", type=Path, default=DEFAULT_CACHE_DIR)
    parser.add_argument("--results-dir", type=Path, default=DEFAULT_RESULTS_DIR)
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="firm-eskf",
        description="Replay the production STM32 ESKF over decoded FIRM launch datasets.",
    )
    _add_common_paths(parser)
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list", help="list usable launch datasets")
    list_parser.set_defaults(handler=_command_list)

    prepare_parser = subparsers.add_parser("prepare", help="cache and align launch CSV files")
    prepare_parser.add_argument(
        "datasets", nargs="*", help="dataset names or paths; defaults to all"
    )
    prepare_parser.add_argument(
        "--force", action="store_true", help="rebuild an existing valid cache"
    )
    prepare_parser.set_defaults(handler=_command_prepare)

    run_parser = subparsers.add_parser("run", help="compile and run the ESKF over launch data")
    run_parser.add_argument("datasets", nargs="*", help="dataset names or paths; defaults to all")
    run_parser.add_argument("--force-prepare", action="store_true", help="rebuild input caches")
    run_parser.add_argument(
        "--force",
        "--skip-native-tests",
        dest="skip_native_tests",
        action="store_true",
        help="build and replay even when the native ESKF tests would fail",
    )
    run_parser.set_defaults(handler=_command_run)

    test_parser = subparsers.add_parser("test", help="build and run native ESKF tests")
    test_parser.set_defaults(handler=_command_test)

    inspect_parser = subparsers.add_parser("inspect", help="show columns, metadata, and metrics")
    inspect_parser.add_argument("result", help="result file, result directory, or dataset name")
    inspect_parser.set_defaults(handler=_command_inspect)

    plot_parser = subparsers.add_parser("plot", help="write a standalone interactive HTML report")
    plot_parser.add_argument("result", help="result file, result directory, or dataset name")
    plot_parser.add_argument("--columns", nargs="+", help="columns to graph")
    plot_parser.add_argument("--output", type=Path, help="HTML output path")
    plot_parser.add_argument("--max-points", type=int, default=100_000)
    plot_parser.add_argument("--absolute-time", action="store_true")
    plot_parser.add_argument("--open", action="store_true", help="open the report in a browser")
    plot_parser.set_defaults(handler=_command_plot)

    compare_parser = subparsers.add_parser("compare", help="overlay columns from multiple runs")
    compare_parser.add_argument("results", nargs="+", help="two or more runs or dataset names")
    compare_parser.add_argument("--columns", nargs="+", help="columns to graph")
    compare_parser.add_argument("--output", type=Path, default=Path("eskf-comparison.html"))
    compare_parser.add_argument("--max-points", type=int, default=100_000)
    compare_parser.add_argument("--absolute-time", action="store_true")
    compare_parser.add_argument("--open", action="store_true")
    compare_parser.set_defaults(handler=_command_compare)

    serve_parser = subparsers.add_parser("serve", help="start the column-selectable dashboard")
    serve_parser.add_argument(
        "result",
        nargs="?",
        help="result file, result directory, or dataset name; defaults to all latest runs",
    )
    serve_parser.add_argument("--host", default="127.0.0.1")
    serve_parser.add_argument("--port", type=int, default=8050)
    serve_parser.add_argument("--debug", action="store_true")
    serve_parser.set_defaults(handler=_command_serve)
    return parser


def _dataset_paths(
    values: Sequence[str], args: argparse.Namespace, profile: DatasetProfile
) -> list[Path]:
    if values:
        return [resolve_dataset(value, args.datasets_dir, profile) for value in values]
    paths = discover_datasets(args.datasets_dir, profile)
    if not paths:
        raise FileNotFoundError(f"no datasets found under {args.datasets_dir.resolve()}")
    return paths


def _prepare(
    path: Path, args: argparse.Namespace, profile: DatasetProfile, *, force: bool
) -> PreparedDataset:
    print(f"Preparing {path.name} ...", flush=True)
    prepared = prepare_dataset(path, profile, args.cache_dir, force=force)
    size = prepared.parquet_path.stat().st_size + prepared.replay_path.stat().st_size
    print(
        f"  {prepared.metadata['rows']:,} replay rows, "
        f"{prepared.metadata['duration_seconds']:.2f} s of data, {human_size(size)} cached"
    )
    return prepared


def _command_list(args: argparse.Namespace, profile: DatasetProfile) -> int:
    paths = discover_datasets(args.datasets_dir, profile)
    if not paths:
        print(f"No complete datasets found under {args.datasets_dir.resolve()}")
        return 0
    for path in paths:
        total_size = sum(
            (path / sensor.filename).stat().st_size for sensor in profile.sensors.values()
        )
        print(f"{path.name:<32} {human_size(total_size):>12}  {path}")
    return 0


def _command_prepare(args: argparse.Namespace, profile: DatasetProfile) -> int:
    for path in _dataset_paths(args.datasets, args, profile):
        _prepare(path, args, profile, force=args.force)
    return 0


def _command_run(args: argparse.Namespace, profile: DatasetProfile) -> int:
    paths = _dataset_paths(args.datasets, args, profile)
    print("Building the native production-source ESKF ...", flush=True)
    if args.skip_native_tests:
        print("  WARNING: native ESKF tests bypassed by --force", flush=True)
    executable = build_native(args.build_dir, run_tests=not args.skip_native_tests)
    print(f"  {executable}")
    for path in paths:
        prepared = _prepare(path, args, profile, force=args.force_prepare)
        print(f"Replaying {path.name} ...", flush=True)
        result = run_replay(prepared, executable, args.results_dir)
        metrics = json.loads(result.metrics_path.read_text())
        rate = metrics.get("rows_per_second")
        rate_text = f" at {rate:,.0f} rows/s" if isinstance(rate, (float, int)) else ""
        print(f"  wrote {result.parquet_path} ({metrics['rows']:,} rows{rate_text})")
        if metrics.get("nonfinite_state_rows"):
            print(
                f"  WARNING: {metrics['nonfinite_state_rows']:,} rows contain "
                "non-finite state values"
            )
    return 0


def _command_test(args: argparse.Namespace, _profile: DatasetProfile) -> int:
    executable = build_native(args.build_dir, run_tests=True)
    print(f"Native ESKF tests passed: {executable}")
    return 0


def _resolve_result(value: str, results_dir: Path) -> Path:
    path = Path(value).expanduser()
    if path.is_file():
        return path.resolve()
    if path.is_dir() and (path / "result.parquet").is_file():
        return (path / "result.parquet").resolve()
    dataset_dir = results_dir.resolve() / value
    latest_path = dataset_dir / "latest.json"
    if latest_path.is_file():
        latest = json.loads(latest_path.read_text())
        result = Path(latest["result_directory"]) / "result.parquet"
        if result.is_file():
            return result.resolve()
    raise FileNotFoundError(f"could not resolve result {value!r}")


def _latest_results(results_dir: Path) -> dict[str, Path]:
    """Resolve the newest replay for every dataset that has one."""
    root = results_dir.resolve()
    if not root.is_dir():
        raise FileNotFoundError(f"results directory does not exist: {root}")
    results: dict[str, Path] = {}
    for dataset_dir in sorted(root.iterdir(), key=lambda item: item.name.casefold()):
        if not dataset_dir.is_dir() or not (dataset_dir / "latest.json").is_file():
            continue
        results[dataset_dir.name] = _resolve_result(dataset_dir.name, root)
    if not results:
        raise FileNotFoundError(f"no completed ESKF results found under {root}")
    return results


def _validated_columns(paths: Sequence[Path], requested: Sequence[str] | None) -> list[str]:
    if requested:
        missing = {
            column: [str(path) for path in paths if column not in result_columns(path)]
            for column in requested
        }
        missing = {column: locations for column, locations in missing.items() if locations}
        if missing:
            raise ValueError(
                f"requested columns are missing from one or more runs: {', '.join(missing)}"
            )
        return list(requested)
    defaults = select_default_columns(paths[0])
    common = set.intersection(*(set(result_columns(path)) for path in paths))
    return [column for column in defaults if column in common]


def _command_inspect(args: argparse.Namespace, _profile: DatasetProfile) -> int:
    path = _resolve_result(args.result, args.results_dir)
    print(path)
    schema = pl.scan_parquet(path).collect_schema()
    print("\nColumns:")
    for name, dtype in schema.items():
        print(f"  {name:<38} {dtype}")
    for filename, title in (("metrics.json", "Metrics"), ("run.json", "Run metadata")):
        metadata_path = path.parent / filename
        if metadata_path.is_file():
            print(f"\n{title}:")
            print(metadata_path.read_text().rstrip())
    return 0


def _command_plot(args: argparse.Namespace, _profile: DatasetProfile) -> int:
    path = _resolve_result(args.result, args.results_dir)
    columns = _validated_columns([path], args.columns)
    output = args.output or path.parent / "report.html"
    written = write_report(
        [path],
        columns,
        output,
        max_points=args.max_points,
        relative_time=not args.absolute_time,
        open_browser=args.open,
    )
    print(f"Wrote {written}")
    return 0


def _command_compare(args: argparse.Namespace, _profile: DatasetProfile) -> int:
    if len(args.results) < 2:
        raise ValueError("compare requires at least two results")
    paths = [_resolve_result(value, args.results_dir) for value in args.results]
    columns = _validated_columns(paths, args.columns)
    written = write_report(
        paths,
        columns,
        args.output,
        max_points=args.max_points,
        relative_time=not args.absolute_time,
        open_browser=args.open,
    )
    print(f"Wrote {written}")
    return 0


def _command_serve(args: argparse.Namespace, _profile: DatasetProfile) -> int:
    results = (
        {args.result: _resolve_result(args.result, args.results_dir)}
        if args.result
        else _latest_results(args.results_dir)
    )
    print(f"Serving {len(results)} dataset(s) at http://{args.host}:{args.port}")
    serve_results(results, host=args.host, port=args.port, debug=args.debug)
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    """Run the ESKF lab CLI."""
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        profile = load_profile(args.profile)
        return int(args.handler(args, profile))
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
