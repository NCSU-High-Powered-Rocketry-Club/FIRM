import threading
import time


def calibrate_magnetometer(firm, calibration_duration_seconds=180, outlier_percentage=0.02):
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
    collected_packets = []
    stop_event = threading.Event()

    collector_thread = threading.Thread(
        target=_packet_collector,
        args=(stop_event, firm, collected_packets),
        name="FIRM-Calibrator",
        daemon=True,
    )
    collector_thread.start()

    # Countdown every 5 seconds, printing the remainder on the last tick
    update_interval = 5
    remaining = int(calibration_duration_seconds)
    while remaining > 0:
        print(f"[Calibration] {remaining} seconds remaining...")
        sleep_time = update_interval if remaining >= update_interval else remaining
        time.sleep(sleep_time)
        remaining -= sleep_time

    # Stop the collector and wait for it to finish
    stop_event.set()
    collector_thread.join()

    # Final non-blocking drain to catch any last packets
    leftover = firm.get_data_packets(block=False)
    if leftover:
        collected_packets.extend(leftover)

    print(f"[Calibration] Finished! Collected {len(collected_packets)} packets.")

    # Extract magnetometer triplets from packets
    x_vals, y_vals, z_vals = [], [], []
    for p in collected_packets:
        x_vals.append(p.mag_x_microteslas)
        y_vals.append(p.mag_y_microteslas)
        z_vals.append(p.mag_z_microteslas)

    if len(x_vals) == 0:
        raise ValueError("No magnetometer samples collected during calibration.")

    # Outlier filtering, then offsets and scales
    fx, fy, fz = _filter_outliers_xyz(x_vals, y_vals, z_vals, outlier_percentage)
    offset_x, offset_y, offset_z, scale_x, scale_y, scale_z = _compute_offsets_and_scales(
        fx, fy, fz
    )

    print(f"[Calibration] Offsets (µT): ({offset_x:.3f}, {offset_y:.3f}, {offset_z:.3f})")
    print(f"[Calibration] Scales  (-):  ({scale_x:.6f}, {scale_y:.6f}, {scale_z:.6f})")

    # Return constants in a simple list, as requested
    return [offset_x, offset_y, offset_z, scale_x, scale_y, scale_z]


def _packet_collector(stop_event, firm, collected_packets):
    """
    Continuously retrieve packets until stop_event is set.
    Non-blocking drain with a tiny sleep to avoid busy-waiting.
    """
    while not stop_event.is_set():
        packets = firm.get_data_packets(block=False)
        if packets:
            collected_packets.extend(packets)
        time.sleep(0.01)


def _percentile_trim_indices(count, outlier_percentage):
    """
    Compute symmetric trim start/end indices for a sorted list length 'count'.
    Returns (start_index, end_index_exclusive).
    """
    if count <= 0:
        return 0, 0
    if outlier_percentage < 0:
        outlier_percentage = 0.0
    if outlier_percentage > 0.98:
        outlier_percentage = 0.98  # keep at least 2% of data

    tail = outlier_percentage / 2.0
    start = int(count * tail)
    end = count - int(count * tail)
    if start >= end:
        # Ensure at least one element remains
        start = 0
        end = count
    return start, end


def _filter_outliers_xyz(x_vals, y_vals, z_vals, outlier_percentage):
    """
    Symmetric per-axis quantile clipping without numpy.
    Keeps rows that fall within the retained range on ALL three axes.
    Returns filtered (x_list, y_list, z_list).
    """
    n = len(x_vals)
    if n == 0:
        return [], [], []

    # Prepare sorted copies to determine bounds
    xs = sorted(x_vals)
    ys = sorted(y_vals)
    zs = sorted(z_vals)

    sx, ex = _percentile_trim_indices(len(xs), outlier_percentage)
    sy, ey = _percentile_trim_indices(len(ys), outlier_percentage)
    sz, ez = _percentile_trim_indices(len(zs), outlier_percentage)

    low_x, high_x = xs[sx], xs[ex - 1]
    low_y, high_y = ys[sy], ys[ey - 1]
    low_z, high_z = zs[sz], zs[ez - 1]

    fx, fy, fz = [], [], []
    for i in range(n):
        vx = x_vals[i]
        vy = y_vals[i]
        vz = z_vals[i]
        if (low_x <= vx <= high_x) and (low_y <= vy <= high_y) and (low_z <= vz <= high_z):
            fx.append(vx)
            fy.append(vy)
            fz.append(vz)

    # If we filtered too aggressively, fall back to original data
    if len(fx) < 8:
        return x_vals, y_vals, z_vals

    return fx, fy, fz


def _compute_offsets_and_scales(x_vals, y_vals, z_vals):
    """
    Mirrors the reference logic:
      - Offsets are midpoints of per-axis min/max.
      - Scales make each axis half-range match the average half-range.
    Returns (offset_x, offset_y, offset_z, scale_x, scale_y, scale_z).
    """
    if len(x_vals) == 0:
        raise ValueError("No samples available for calibration.")

    min_x, max_x = min(x_vals), max(x_vals)
    min_y, max_y = min(y_vals), max(y_vals)
    min_z, max_z = min(z_vals), max(z_vals)

    offset_x = (max_x + min_x) / 2.0
    offset_y = (max_y + min_y) / 2.0
    offset_z = (max_z + min_z) / 2.0

    # Correct values to compute half-ranges
    cx = [v - offset_x for v in x_vals]
    cy = [v - offset_y for v in y_vals]
    cz = [v - offset_z for v in z_vals]

    hr_x = (max(cx) - min(cx)) / 2.0
    hr_y = (max(cy) - min(cy)) / 2.0
    hr_z = (max(cz) - min(cz)) / 2.0

    # Average half-range
    average_half_range = (hr_x + hr_y + hr_z) / 3.0

    def safe_scale(half_range):
        if half_range == 0.0:
            return 1.0
        return average_half_range / half_range

    scale_x = safe_scale(hr_x)
    scale_y = safe_scale(hr_y)
    scale_z = safe_scale(hr_z)

    return offset_x, offset_y, offset_z, scale_x, scale_y, scale_z
