import threading
import time
from typing import List

from python.firm import FIRM, FIRMPacket


def calibrate_magnetometer(firm: FIRM, calibration_duration_seconds: int = 180) -> None:
    """
    Calibrates FIRM's magnetometer for the specified duration. During calibration, the function
    continuously collects data packets from FIRM in a separate thread. FIRM should be physically
    rotated in all orientations during this period to ensure a comprehensive calibration.

    Args:
        firm: An initialized FIRM instance.
        calibration_duration_seconds: Duration for calibration in seconds (default is 180 seconds).

    Returns:
        A list of calibration constants.
    """
    collected_packets: List[FIRMPacket] = []
    stop_event = threading.Event()

    def packet_collector() -> None:
        """Continuously retrieve packets until stop_event is set."""
        while not stop_event.is_set():
            packets = firm.get_data_packets()
            if packets:
                collected_packets.extend(packets)

    collector_thread = threading.Thread(
        target=packet_collector,
        name="FIRM-Calibrator",
        daemon=True,
    )
    collector_thread.start()

    update_interval = 5
    for remaining in range(calibration_duration_seconds, 0, -update_interval):
        print(f"[calibration] {remaining} seconds remaining...")
        time.sleep(update_interval)

    # Signal stop and wait for the collector to finish
    stop_event.set()
    collector_thread.join()

    print(f"[calibration] Finished! Collected {len(collected_packets)} packets.")
