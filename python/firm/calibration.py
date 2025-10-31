import threading


def calibrate_magnetometer(firm, calibration_duration_seconds=180):
    """
    Calibrates FIRM's magnetometer for the specified duration. During calibration, the function
    continuously collects data packets from FIRM. FIRM should be physically rotated in all
    orientations during this period to ensure a comprehensive calibration.

    Args:
        firm: An initialized FIRM instance.
        calibration_duration_seconds: Duration for calibration in seconds (default is 180 seconds).

    Returns:
        A list of calibration constants.
    """
    print("[Calibration] Collecting packets...")
    collected_packets = []

    # Schedule countdown printouts (fires asynchronously)
    _schedule_countdown_timers(calibration_duration_seconds)

    # Compute absolute end time for the loop
    end_time = threading.Timer(calibration_duration_seconds, lambda: None)
    end_time.start()

    # Continuous blocking reads until all timers finish
    while end_time.is_alive():
        packets = firm.get_data_packets()
        if packets:
            collected_packets.extend(packets)

    print(f"[Calibration] Finished! Collected {len(collected_packets)} packets.")

    x_vals, y_vals, z_vals = [], [], []
    for p in collected_packets:
        x_vals.append(p.mag_x_microteslas)
        y_vals.append(p.mag_y_microteslas)
        z_vals.append(p.mag_z_microteslas)

    if len(x_vals) == 0:
        raise ValueError("No magnetometer samples collected during calibration.")

    outlier_percentage = 0.02
    fx, fy, fz = _filter_outliers_xyz(x_vals, y_vals, z_vals, outlier_percentage)
    offset_x, offset_y, offset_z, scale_x, scale_y, scale_z = _compute_offsets_and_scales(fx, fy, fz)

    print(f"[Calibration] Offsets (µT): ({offset_x:.3f}, {offset_y:.3f}, {offset_z:.3f})")
    print(f"[Calibration] Scales  (-):  ({scale_x:.6f}, {scale_y:.6f}, {scale_z:.6f})")

    return [offset_x, offset_y, offset_z, scale_x, scale_y, scale_z]


def _schedule_countdown_timers(total_seconds, interval=5):
    """
    Schedule countdown printouts using threading.Timer.
    Fires messages at fixed intervals without recursion or nesting.
    """
    for remaining in range(total_seconds, 0, -interval):
        delay = total_seconds - remaining
        timer = threading.Timer(delay, print, args=(f"[Calibration] {remaining} seconds remaining...",))
        timer.daemon = True
        timer.start()


def _filter_outliers_xyz(x_vals, y_vals, z_vals, outlier_percentage):
    """Symmetric percentile trimming per axis (same as the GitHub reference)."""
    def trim(values):
        sorted_vals = sorted(values)
        n = len(sorted_vals)
        tail = outlier_percentage / 2.0
        start = int(n * tail)
        end = n - int(n * tail)
        return sorted_vals[start:end]

    trimmed_x = trim(x_vals)
    trimmed_y = trim(y_vals)
    trimmed_z = trim(z_vals)

    if len(trimmed_x) < 8:
        return x_vals, y_vals, z_vals
    return trimmed_x, trimmed_y, trimmed_z


def _compute_offsets_and_scales(x_vals, y_vals, z_vals):
    """Compute per-axis offsets and scale factors."""
    min_x, max_x = min(x_vals), max(x_vals)
    min_y, max_y = min(y_vals), max(y_vals)
    min_z, max_z = min(z_vals), max(z_vals)

    offset_x = (max_x + min_x) / 2.0
    offset_y = (max_y + min_y) / 2.0
    offset_z = (max_z + min_z) / 2.0

    cx = [v - offset_x for v in x_vals]
    cy = [v - offset_y for v in y_vals]
    cz = [v - offset_z for v in z_vals]

    hr_x = (max(cx) - min(cx)) / 2.0
    hr_y = (max(cy) - min(cy)) / 2.0
    hr_z = (max(cz) - min(cz)) / 2.0
    avg_half_range = (hr_x + hr_y + hr_z) / 3.0

    def safe_scale(half_range):
        if half_range == 0.0:
            return 1.0
        return avg_half_range / half_range

    scale_x = safe_scale(hr_x)
    scale_y = safe_scale(hr_y)
    scale_z = safe_scale(hr_z)

    return offset_x, offset_y, offset_z, scale_x, scale_y, scale_z

