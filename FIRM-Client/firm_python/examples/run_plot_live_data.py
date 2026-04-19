from __future__ import annotations

import sys
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Tuple

from firm_client import FIRMClient

DEFAULT_PORT = "COM12"
DEFAULT_BAUD_RATE = 2_000_000

FIELDS: List[str] = [
    "raw_acceleration_x_gs",
    "raw_acceleration_y_gs",
    "raw_acceleration_z_gs",
    "raw_angular_rate_x_deg_per_s",
    "raw_angular_rate_y_deg_per_s",
    "raw_angular_rate_z_deg_per_s",
]

# Plot refresh interval.
REFRESH_MS = 50

# Keep a rolling buffer; older points are dropped.
MAX_POINTS = 4_000

USE_PACKET_TIMESTAMP = True

KALMAN_FIELDS: List[str] = [
    "est_position_z_meters",
    "est_velocity_z_meters_per_s",
    "est_quaternion_w",
    "est_quaternion_x",
    "est_quaternion_y",
    "est_quaternion_z",
]

# Treat values as "unchanged" if within this tolerance.
KALMAN_CHANGE_EPS = 1e-12

# Compute Hz over a rolling time window.
KALMAN_WINDOW_SECONDS = 5.0


@dataclass(frozen=True)
class Series:
    x: Deque[float]
    y: Deque[float]


def _try_get_float(obj: object, attr: str) -> Optional[float]:
    value = getattr(obj, attr, None)
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _parse_args(argv: List[str]) -> Tuple[str, int]:
    port = DEFAULT_PORT
    baud = DEFAULT_BAUD_RATE

    if len(argv) >= 2:
        port = argv[1]
    if len(argv) >= 3:
        baud = int(argv[2])

    return port, baud


def main(argv: List[str]) -> int:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except Exception as exc:  # pragma: no cover
        print(
            "matplotlib is required for this example. Install with: uv add matplotlib\n"
            f"Import error: {exc}",
            file=sys.stderr,
        )
        return 2

    port, baud_rate = _parse_args(argv)

    # One series per field.
    series_by_field: Dict[str, Series] = {
        field: Series(x=deque(maxlen=MAX_POINTS), y=deque(maxlen=MAX_POINTS))
        for field in FIELDS
    }

    start_packet_ts: Optional[float] = None
    start_wall_ts = time.time()

    kalman_last_snapshot: Optional[Dict[str, float]] = None
    kalman_update_times: Deque[float] = deque()
    kalman_total_updates = 0

    # Matplotlib setup.
    n = max(1, len(FIELDS))
    fig, axes = plt.subplots(n, 1, sharex=True, figsize=(10, max(4, 2.2 * n)))
    if n == 1:
        axes_list = [axes]
    else:
        axes_list = list(axes)

    lines = {}
    for ax, field in zip(axes_list, FIELDS):
        (line,) = ax.plot([], [], lw=1)
        lines[field] = line
        ax.set_ylabel(field)
        ax.grid(True, alpha=0.25)

    if axes_list:
        axes_list[-1].set_xlabel("t (s)")

    # Light status text.
    status = fig.text(0.01, 0.99, "", ha="left", va="top")

    with FIRMClient(port, baud_rate) as client:
        # Clear initial packets (blocking once).
        try:
            client.get_data_packets(block=True)
        except TypeError:
            # In case bindings don't support the kwarg in some versions.
            client.get_data_packets(True)

        last_rx_count = 0
        last_status_time = 0.0

        def _drain_packets() -> int:
            nonlocal start_packet_ts, kalman_last_snapshot, kalman_total_updates
            drained = 0
            try:
                packets = client.get_data_packets(block=False)
            except TypeError:
                packets = client.get_data_packets(False)

            for pkt in packets:
                drained += 1

                pkt_ts = None
                if USE_PACKET_TIMESTAMP:
                    pkt_ts = _try_get_float(pkt, "timestamp_seconds")

                if USE_PACKET_TIMESTAMP:
                    if pkt_ts is None:
                        # Fall back to wall clock if packet timestamp is missing.
                        x = time.time() - start_wall_ts
                    else:
                        if start_packet_ts is None:
                            start_packet_ts = pkt_ts
                        x = pkt_ts - start_packet_ts
                else:
                    x = time.time() - start_wall_ts

                # Kalman update detection (based on estimated fields changing).
                # Use packet timestamp when available; otherwise wall clock.
                kalman_time = pkt_ts if pkt_ts is not None else time.time()
                current_snapshot: Dict[str, float] = {}
                for field in KALMAN_FIELDS:
                    value = _try_get_float(pkt, field)
                    if value is None:
                        continue
                    current_snapshot[field] = value

                if current_snapshot:
                    is_update = False
                    if kalman_last_snapshot is None:
                        # First snapshot doesn't count as an "update", just establishes baseline.
                        kalman_last_snapshot = current_snapshot
                    else:
                        # If any tracked field changes, count as a Kalman update.
                        for field, new_value in current_snapshot.items():
                            old_value = kalman_last_snapshot.get(field)
                            if old_value is None:
                                is_update = True
                                break
                            if abs(new_value - old_value) > KALMAN_CHANGE_EPS:
                                is_update = True
                                break

                        kalman_last_snapshot = current_snapshot

                    if is_update:
                        kalman_total_updates += 1
                        kalman_update_times.append(kalman_time)
                        # Trim to rolling window.
                        while (
                            kalman_update_times
                            and kalman_time - kalman_update_times[0]
                            > KALMAN_WINDOW_SECONDS
                        ):
                            kalman_update_times.popleft()

                for field, s in series_by_field.items():
                    y = _try_get_float(pkt, field)
                    if y is None:
                        continue
                    s.x.append(x)
                    s.y.append(y)

            return drained

        def _update(_frame: int):
            nonlocal last_rx_count, last_status_time

            rx = _drain_packets()
            last_rx_count += rx

            # Update plot lines.
            for field, s in series_by_field.items():
                line = lines.get(field)
                if line is None:
                    continue
                line.set_data(list(s.x), list(s.y))

            # Autoscale.
            for ax in axes_list:
                ax.relim()
                ax.autoscale_view(scalex=True, scaley=True)

            now = time.time()
            if now - last_status_time > 0.25:
                last_status_time = now

                # Estimate Kalman Hz from timestamps in the rolling window.
                kalman_hz = 0.0
                if len(kalman_update_times) >= 2:
                    dt = kalman_update_times[-1] - kalman_update_times[0]
                    if dt > 0:
                        kalman_hz = (len(kalman_update_times) - 1) / dt

                status.set_text(
                    f"Port={port}  Baud={baud_rate}  Fields={len(FIELDS)}  "
                    f"Total packets={last_rx_count}  "
                    f"Kalman≈{kalman_hz:.2f} Hz (updates={kalman_total_updates}, window={KALMAN_WINDOW_SECONDS:.0f}s)"
                )

            return list(lines.values()) + [status]

        anim = FuncAnimation(fig, _update, interval=REFRESH_MS, blit=False)

        # Keep reference to avoid GC of animation.
        _ = anim
        plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
