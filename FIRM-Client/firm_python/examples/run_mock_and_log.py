import argparse
import csv
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from firm_client import FIRMClient


# uv run  .\firm_python\examples\run_mock_and_log.py --out output.csv COM8 "C:\Users\jackg\Downloads\gov_work_launch_1_ab (1).FRM"

TIMEOUT_SECONDS_DEFAULT = 0.5
START_TIMEOUT_SECONDS_DEFAULT = 5.0
REALTIME_DEFAULT = True
SPEED_DEFAULT = 1.0
CHUNK_SIZE_DEFAULT = 80_000
DRAIN_SECONDS_DEFAULT = 1.0


FIELDS: List[str] = [
    "timestamp_seconds",
    "temperature_celsius",
    "pressure_pascals",
    "raw_acceleration_x_gs",
    "raw_acceleration_y_gs",
    "raw_acceleration_z_gs",
    "raw_angular_rate_x_deg_per_s",
    "raw_angular_rate_y_deg_per_s",
    "raw_angular_rate_z_deg_per_s",
    "magnetic_field_x_microteslas",
    "magnetic_field_y_microteslas",
    "magnetic_field_z_microteslas",
    "est_position_z_meters",
    "est_velocity_z_meters_per_s",
    "est_quaternion_w",
    "est_quaternion_x",
    "est_quaternion_y",
    "est_quaternion_z",
]


def _row_from_packet(pkt: Any) -> Dict[str, Any]:
    return {f: getattr(pkt, f) for f in FIELDS}


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Stream a mock FIRM log file asynchronously and save FIRM output packets to CSV"
    )
    parser.add_argument(
        "port", help='Serial port name (e.g., "COM8" or "/dev/ttyACM0")'
    )
    parser.add_argument("log_path", help="Path to the FIRM log file to stream")
    parser.add_argument("--out", required=True, help="Output CSV path")
    parser.add_argument(
        "-b",
        "--baud-rate",
        type=int,
        default=2_000_000,
        help="Baud rate (default: 2000000)",
    )
    parser.add_argument(
        "--timeout-seconds",
        type=float,
        default=TIMEOUT_SECONDS_DEFAULT,
        help=f"Timeout used by get_data_packets(block=True) (default: {TIMEOUT_SECONDS_DEFAULT})",
    )
    parser.add_argument(
        "--start-timeout-seconds",
        type=float,
        default=START_TIMEOUT_SECONDS_DEFAULT,
        help=f"Timeout waiting for mock ack (default: {START_TIMEOUT_SECONDS_DEFAULT})",
    )
    parser.add_argument(
        "--realtime",
        action=argparse.BooleanOptionalAction,
        default=REALTIME_DEFAULT,
        help=f"Pace the stream by timestamps (default: {REALTIME_DEFAULT})",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=SPEED_DEFAULT,
        help=f"Playback speed multiplier when realtime (default: {SPEED_DEFAULT})",
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=CHUNK_SIZE_DEFAULT,
        help=f"File read chunk size (default: {CHUNK_SIZE_DEFAULT})",
    )
    parser.add_argument(
        "--drain-seconds",
        type=float,
        default=DRAIN_SECONDS_DEFAULT,
        help=f"How long to keep logging after stream ends (default: {DRAIN_SECONDS_DEFAULT})",
    )
    args = parser.parse_args()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    total_rows = 0
    sent_packets: Optional[int] = None
    mock_finished_wall: Optional[float] = None

    start_wall = time.time()

    with FIRMClient(args.port, args.baud_rate, args.timeout_seconds) as client:
        # Best-effort drain (doesn't matter if empty)
        try:
            client.get_data_packets(block=True)
        except Exception:
            pass

        print("Starting async mock stream...")
        client.start_mock_log_stream(
            args.log_path,
            realtime=args.realtime,
            speed=args.speed,
            chunk_size=args.chunk_size,
            start_timeout_seconds=args.start_timeout_seconds,
            cancel_on_finish=True,
        )

        try:
            with out_path.open("w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=FIELDS)
                writer.writeheader()

                while True:
                    # Read device output packets
                    packets = client.get_data_packets(block=False)
                    for pkt in packets:
                        writer.writerow(_row_from_packet(pkt))
                        total_rows += 1

                    # Detect stream end
                    if not client.is_mock_log_streaming():
                        if mock_finished_wall is None:
                            mock_finished_wall = time.time()

                    # Drain window after stream ends
                    if mock_finished_wall is not None:
                        if time.time() - mock_finished_wall >= float(
                            args.drain_seconds
                        ):
                            break

                    time.sleep(0.001)

        except KeyboardInterrupt:
            print("\nInterrupted — stopping mock stream...")

        finally:
            # Ensure the mock stream is stopped and (optionally) joined.
            # join=True blocks until the mock thread exits and returns packet count.
            try:
                sent_packets = client.stop_mock_log_stream(
                    cancel_device=True, join=True
                )
            except Exception:
                # If something goes wrong (e.g. already stopped), just proceed.
                sent_packets = sent_packets

    elapsed = time.time() - start_wall
    print(f"Wrote {total_rows} FIRM output packets to: {out_path}")
    if sent_packets is not None:
        print(f"Mock packets sent: {sent_packets}")
    print(f"Total time: {elapsed:.2f} seconds")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
