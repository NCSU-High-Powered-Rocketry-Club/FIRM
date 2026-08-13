"""Responsive Plotly reports and a lazy-loading Dash explorer."""

from __future__ import annotations

import itertools
import json
import webbrowser
from typing import TYPE_CHECKING

import numpy as np
import plotly.graph_objects as go
import polars as pl
from plotly.subplots import make_subplots

if TYPE_CHECKING:
    from collections.abc import Sequence
    from pathlib import Path

DEFAULT_COLUMNS = (
    "raw_baro_altitude_m",
    "eskf_position_z_m",
    "eskf_velocity_z_mps",
    "imu_accel_z_g",
    "high_g_accel_z_g",
    "eskf_pressure_coupling",
)
ALTITUDE_COLUMNS = ("raw_baro_altitude_m", "eskf_position_z_m")
DATASET_TAB_STYLE = {
    "boxSizing": "border-box",
    "flex": "1 1 240px",
    "minWidth": "240px",
    "padding": "10px 14px",
    "height": "auto",
    "lineHeight": "1.25",
    "textAlign": "center",
    "whiteSpace": "normal",
}
SELECTED_DATASET_TAB_STYLE = {
    **DATASET_TAB_STYLE,
    "borderTop": "3px solid #119DFF",
    "fontWeight": "600",
}
DATASET_TABS_STYLE = {
    "display": "flex",
    "flexWrap": "wrap",
    "alignItems": "stretch",
    "gap": "4px",
}


def result_columns(path: Path) -> list[str]:
    """Return numeric result columns without loading the dataset."""
    schema = pl.scan_parquet(path).collect_schema()
    return [name for name, dtype in schema.items() if name != "timestamp" and dtype.is_numeric()]


def select_default_columns(path: Path) -> list[str]:
    """Choose useful default raw-versus-filter traces."""
    available = set(result_columns(path))
    return [column for column in DEFAULT_COLUMNS if column in available]


def _minmax_indices(values: np.ndarray, max_points: int) -> np.ndarray:
    count = len(values)
    if max_points <= 0 or count <= max_points:
        return np.arange(count)
    bucket_count = max(1, (max_points - 2) // 2)
    edges = np.linspace(0, count, bucket_count + 1, dtype=np.int64)
    indices: list[int] = [0]
    for start, end in itertools.pairwise(edges):
        if end <= start:
            continue
        bucket = values[start:end]
        finite = np.isfinite(bucket)
        if not finite.any():
            indices.append(int(start))
            continue
        finite_indices = np.flatnonzero(finite)
        minimum = int(start + finite_indices[np.argmin(bucket[finite])])
        maximum = int(start + finite_indices[np.argmax(bucket[finite])])
        indices.extend(sorted((minimum, maximum)))
    indices.append(count - 1)
    return np.unique(indices)


def _subplot_groups(columns: Sequence[str]) -> list[tuple[str, list[str]]]:
    """Group raw and estimated altitude so they can be compared on one axis."""
    selected = set(columns)
    combine_altitude = all(column in selected for column in ALTITUDE_COLUMNS)
    groups: list[tuple[str, list[str]]] = []
    consumed: set[str] = set()
    for column in columns:
        if column in consumed:
            continue
        if combine_altitude and column in ALTITUDE_COLUMNS:
            groups.append(("Altitude (m)", list(ALTITUDE_COLUMNS)))
            consumed.update(ALTITUDE_COLUMNS)
        else:
            groups.append((column, [column]))
            consumed.add(column)
    return groups


def make_figure(
    paths: Sequence[Path],
    columns: Sequence[str],
    *,
    max_points: int = 100_000,
    relative_time: bool = True,
) -> go.Figure:
    """Build linked subplots, overlaying multiple runs when supplied."""
    if not columns:
        raise ValueError("at least one result column must be selected")
    groups = _subplot_groups(columns)
    figure = make_subplots(
        rows=len(groups),
        cols=1,
        shared_xaxes=True,
        vertical_spacing=min(0.04, 0.2 / len(groups)),
        subplot_titles=[title for title, _columns in groups],
    )
    for path in paths:
        run_name = path.parent.name
        available = set(result_columns(path))
        selected = [column for column in columns if column in available]
        frame = pl.read_parquet(path, columns=["timestamp", *selected])
        timestamps = frame["timestamp"].to_numpy()
        time_origin = timestamps[0] if relative_time and timestamps.size else 0.0
        for row, (title, group_columns) in enumerate(groups, start=1):
            for column in group_columns:
                if column not in available:
                    continue
                y = frame[column].to_numpy()
                indices = _minmax_indices(y, max_points)
                x = timestamps[indices] - time_origin
                figure.add_trace(
                    go.Scattergl(
                        x=x,
                        y=y[indices],
                        mode="lines",
                        name=f"{run_name}: {column}",
                        legendgroup=run_name,
                        line={"width": 1.2},
                        showlegend=True,
                    ),
                    row=row,
                    col=1,
                )
            figure.update_yaxes(title_text=title, row=row, col=1)
    figure.update_layout(
        template="plotly_white",
        height=max(500, min(280 * len(groups) + 120, 1800)),
        hovermode="x unified",
        title="FIRM ESKF replay",
        legend={"orientation": "h", "yanchor": "bottom", "y": 1.02},
        margin={"l": 80, "r": 30, "t": 100, "b": 60},
    )
    figure.update_xaxes(
        title_text="time from replay start (s)" if relative_time else "timestamp (s)",
        row=len(groups),
        col=1,
    )
    return figure


def write_report(
    paths: Sequence[Path],
    columns: Sequence[str],
    output: Path,
    *,
    max_points: int,
    relative_time: bool,
    open_browser: bool,
) -> Path:
    """Write a standalone interactive HTML plot."""
    figure = make_figure(paths, columns, max_points=max_points, relative_time=relative_time)
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.write_html(output, include_plotlyjs=True, full_html=True)
    if open_browser:
        webbrowser.open(output.as_uri())
    return output


def serve_results(
    result_paths: dict[str, Path], *, host: str, port: int, debug: bool = False
) -> None:
    """Serve one tabbed dashboard, with a lazy-loading view for each dataset."""
    from dash import Dash, Input, Output, dcc, html

    if not result_paths:
        raise ValueError("at least one result is required to start the dashboard")

    dataset_names = list(result_paths)
    initial_dataset = dataset_names[0]

    def dataset_details(dataset: str):
        path = result_paths[dataset]
        metrics_path = path.parent / "metrics.json"
        metrics = json.loads(metrics_path.read_text()) if metrics_path.is_file() else {}
        return [
            html.P(f"Run: {path.parent.name}"),
            html.Details([html.Summary("Run metrics"), html.Pre(json.dumps(metrics, indent=2))]),
        ]

    initial_path = result_paths[initial_dataset]
    initial_columns = result_columns(initial_path)
    initial_defaults = select_default_columns(initial_path)
    app = Dash(__name__)
    app.title = "FIRM ESKF replay"
    app.layout = html.Div(
        [
            html.H2("FIRM ESKF replay"),
            dcc.Tabs(
                id="dataset-tabs",
                value=initial_dataset,
                style=DATASET_TABS_STYLE,
                parent_style=DATASET_TABS_STYLE,
                mobile_breakpoint=0,
                children=[
                    dcc.Tab(
                        label=dataset,
                        value=dataset,
                        style=DATASET_TAB_STYLE,
                        selected_style=SELECTED_DATASET_TAB_STYLE,
                    )
                    for dataset in dataset_names
                ],
            ),
            html.Div(id="run-details", children=dataset_details(initial_dataset)),
            dcc.Dropdown(
                id="columns",
                options=[{"label": column, "value": column} for column in initial_columns],
                value=initial_defaults,
                multi=True,
                placeholder="Select result or raw-data columns",
            ),
            html.Div(
                [
                    html.Label("Maximum displayed points per trace"),
                    dcc.Input(
                        id="max-points", type="number", value=100_000, min=1_000, step=10_000
                    ),
                ],
                style={"margin": "12px 0"},
            ),
            dcc.Graph(id="graph", config={"scrollZoom": True}),
        ],
        style={"fontFamily": "system-ui", "margin": "20px"},
    )

    @app.callback(
        Output("columns", "options"),
        Output("columns", "value"),
        Output("run-details", "children"),
        Input("dataset-tabs", "value"),
    )
    def select_dataset(dataset: str):
        path = result_paths[dataset]
        columns = result_columns(path)
        return (
            [{"label": column, "value": column} for column in columns],
            select_default_columns(path),
            dataset_details(dataset),
        )

    @app.callback(
        Output("graph", "figure"),
        Input("dataset-tabs", "value"),
        Input("columns", "value"),
        Input("max-points", "value"),
    )
    def update_graph(dataset: str, columns: list[str] | None, max_points: int | None) -> go.Figure:
        path = result_paths[dataset]
        available = set(result_columns(path))
        selected = [column for column in (columns or []) if column in available]
        selected = selected or select_default_columns(path)
        return make_figure([path], selected, max_points=max_points or 100_000)

    app.run(host=host, port=port, debug=debug)
