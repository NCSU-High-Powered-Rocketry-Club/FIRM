import sys
import pandas as pd
from dash import Dash, dcc, html, Input, Output
import plotly.graph_objects as go

DEFAULT_X = "timestamp_seconds"


def is_numeric_series(s: pd.Series) -> bool:
    return pd.api.types.is_numeric_dtype(s)


def downsample_df(df: pd.DataFrame, max_points: int | None) -> pd.DataFrame:
    """Uniformly downsample rows to at most max_points (keeps first/last)."""
    n = len(df)
    if max_points is None or max_points <= 0 or n <= max_points:
        return df
    step = max(1, n // max_points)
    sampled = df.iloc[::step].copy()
    if sampled.index[-1] != df.index[-1]:
        sampled = pd.concat([sampled, df.iloc[[-1]]], axis=0)
    return sampled


def make_figure(
    df: pd.DataFrame, x_col: str, y_cols: list[str], mode: str
) -> go.Figure:
    fig = go.Figure()

    if not y_cols:
        fig.update_layout(
            title="Select one or more columns to plot",
            template="plotly_white",
            height=700,
        )
        return fig

    x = df[x_col]

    if mode == "single":
        # All traces on one set of axes
        for c in y_cols:
            fig.add_trace(go.Scattergl(x=x, y=df[c], mode="lines", name=c))
        fig.update_layout(
            title="Interactive CSV Plot (single chart)",
            template="plotly_white",
            height=700,
            legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="left", x=0),
            margin=dict(l=60, r=30, t=80, b=60),
            xaxis_title=x_col,
        )
        fig.update_yaxes(title_text="Value")
        return fig

    # Subplots mode (stacked)
    from plotly.subplots import make_subplots

    rows = len(y_cols)
    fig = make_subplots(
        rows=rows,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=min(0.08, 0.35 / max(rows, 1)),
        subplot_titles=y_cols,
    )

    for i, c in enumerate(y_cols, start=1):
        fig.add_trace(
            go.Scattergl(x=x, y=df[c], mode="lines", name=c, showlegend=False),
            row=i,
            col=1,
        )
        fig.update_yaxes(title_text=c, row=i, col=1)

    fig.update_layout(
        title="Interactive CSV Plot (stacked subplots)",
        template="plotly_white",
        height=min(250 * rows + 150, 1400),
        margin=dict(l=60, r=30, t=80, b=60),
    )
    fig.update_xaxes(title_text=x_col, row=rows, col=1)
    return fig


def main():
    if len(sys.argv) < 2:
        print("Usage: python plot_csv_dashboard.py path/to/data.csv")
        sys.exit(1)

    csv_path = sys.argv[1]

    # Read CSV
    df = pd.read_csv(csv_path)

    # If timestamp_seconds exists, sort by it (helps zoom/line rendering)
    if DEFAULT_X in df.columns and is_numeric_series(df[DEFAULT_X]):
        df = df.sort_values(DEFAULT_X).reset_index(drop=True)

    # Build list of numeric columns
    numeric_cols = [c for c in df.columns if is_numeric_series(df[c])]

    # Choose default X column
    x_default = (
        DEFAULT_X
        if DEFAULT_X in numeric_cols
        else (numeric_cols[0] if numeric_cols else df.columns[0])
    )

    # Y candidates = numeric columns excluding X
    y_candidates = [c for c in numeric_cols if c != x_default]

    # Some sensible defaults (you can change these)
    default_selected = [
        c
        for c in y_candidates
        if any(
            k in c
            for k in [
                "temperature",
                "pressure",
                "raw_acceleration",
                "velocity",
                "quaternion",
            ]
        )
    ]
    if not default_selected:
        default_selected = y_candidates[:3]

    app = Dash(__name__)
    app.title = "CSV Plotter"

    app.layout = html.Div(
        style={
            "fontFamily": "system-ui, -apple-system, Segoe UI, Roboto, Arial",
            "padding": "16px",
        },
        children=[
            html.H2("Interactive CSV Plotter"),
            html.Div(
                style={
                    "display": "flex",
                    "gap": "16px",
                    "flexWrap": "wrap",
                    "alignItems": "flex-start",
                },
                children=[
                    html.Div(
                        style={"minWidth": "280px", "maxWidth": "420px", "flex": "1"},
                        children=[
                            html.Div("X axis column"),
                            dcc.Dropdown(
                                id="x-col",
                                options=[{"label": c, "value": c} for c in numeric_cols]
                                or [{"label": c, "value": c} for c in df.columns],
                                value=x_default,
                                clearable=False,
                            ),
                            html.Div(style={"height": "12px"}),
                            html.Div("Y columns (checkbox list)"),
                            dcc.Checklist(
                                id="y-cols",
                                options=[
                                    {"label": c, "value": c} for c in y_candidates
                                ],
                                value=default_selected,
                                labelStyle={"display": "block", "margin": "2px 0"},
                                inputStyle={"marginRight": "8px"},
                            ),
                            html.Hr(),
                            html.Div("Display mode"),
                            dcc.RadioItems(
                                id="mode",
                                options=[
                                    {
                                        "label": "Single chart (all lines together)",
                                        "value": "single",
                                    },
                                    {
                                        "label": "Stacked subplots (one per signal)",
                                        "value": "stacked",
                                    },
                                ],
                                value="single",
                                labelStyle={"display": "block", "margin": "4px 0"},
                                inputStyle={"marginRight": "8px"},
                            ),
                            html.Hr(),
                            html.Div("Max points (downsample for speed)"),
                            dcc.Slider(
                                id="max-points",
                                min=500,
                                max=200000,
                                step=500,
                                value=50000,
                                marks={
                                    500: "500",
                                    5000: "5k",
                                    50000: "50k",
                                    200000: "200k",
                                },
                                tooltip={
                                    "placement": "bottom",
                                    "always_visible": False,
                                },
                            ),
                            html.Div(style={"height": "12px"}),
                            dcc.Checklist(
                                id="show-markers",
                                options=[
                                    {
                                        "label": "Show markers (slower for big data)",
                                        "value": "markers",
                                    }
                                ],
                                value=[],
                                labelStyle={"display": "block"},
                                inputStyle={"marginRight": "8px"},
                            ),
                        ],
                    ),
                    html.Div(
                        style={"flex": "3", "minWidth": "520px"},
                        children=[
                            dcc.Graph(
                                id="graph",
                                config={
                                    "displaylogo": False,
                                    "scrollZoom": True,  # mousewheel zoom
                                    "modeBarButtonsToAdd": [
                                        "drawline",
                                        "drawopenpath",
                                        "drawrect",
                                        "eraseshape",
                                    ],
                                },
                                style={"height": "760px"},
                            ),
                            html.Div(
                                id="stats",
                                style={"marginTop": "8px", "color": "#444"},
                            ),
                        ],
                    ),
                ],
            ),
        ],
    )

    @app.callback(
        Output("graph", "figure"),
        Output("stats", "children"),
        Input("x-col", "value"),
        Input("y-cols", "value"),
        Input("mode", "value"),
        Input("max-points", "value"),
        Input("show-markers", "value"),
    )
    def update_graph(x_col, y_cols, mode, max_points, show_markers):
        # Validate selection
        if x_col not in df.columns:
            return go.Figure(), f"X column '{x_col}' not found."

        # Only plot numeric y columns that exist
        y_cols = [
            c for c in (y_cols or []) if c in df.columns and is_numeric_series(df[c])
        ]

        # Downsample for speed
        dff = downsample_df(df, int(max_points) if max_points else None)

        fig = make_figure(
            dff, x_col, y_cols, "single" if mode == "single" else "stacked"
        )

        # Optionally add markers
        if "markers" in (show_markers or []):
            for tr in fig.data:
                tr.mode = "lines+markers"
                tr.marker = dict(size=4)

        # Improve hover
        fig.update_traces(
            hovertemplate=f"{x_col}=%{{x}}<br>%{{y}}<extra>%{{fullData.name}}</extra>"
        )

        # Nice interactions
        fig.update_layout(
            hovermode="x unified" if mode == "single" else "closest",
        )

        return (
            fig,
            f"Loaded rows: {len(df):,}. Displayed rows (downsampled): {len(dff):,}.",
        )

    print("\nStarting server...")
    print("Open the URL shown below in your browser.\n")
    app.run(debug=False, host="127.0.0.1", port=8050)


if __name__ == "__main__":
    main()
