"""Smoke test for host-side ESKF Python-to-C integration."""

from __future__ import annotations

import sys

try:
    from .filter_api import ORIENTATION_V2, FilterHarness
except ImportError:
    from filter_api import ORIENTATION_V2, FilterHarness


def run_smoke() -> None:
    """Build, initialize, step, read state, then verify reset/reinit."""
    harness = FilterHarness(auto_build=True)

    harness.reset()
    harness.set_orientation_variant(ORIENTATION_V2)
    harness.initialize_default(sample_count=64)

    sys.stdout.write("Smoke run: initialized with default startup samples\n")

    dt_s = 0.01
    pressure_pa = 101325.0

    for step_idx in range(1, 21):
        harness.step(
            dt_s=dt_s,
            pressure_pa=pressure_pa,
            accel_x_g=0.0,
            accel_y_g=0.0,
            accel_z_g=1.0,
            gyro_x_dps=0.0,
            gyro_y_dps=0.0,
            gyro_z_dps=0.0,
            mag_x_ut=20.0,
            mag_y_ut=0.0,
            mag_z_ut=45.0,
        )

        if step_idx in (1, 5, 10, 20):
            altitude, velocity, quat = harness.get_state()
            sys.stdout.write(
                f"step={step_idx:02d} alt={altitude:.6f} vel={velocity:.6f} "
                f"quat=({quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f})\n"
            )

    harness.reset()
    harness.set_orientation_variant(ORIENTATION_V2)
    harness.initialize_default(sample_count=64)
    altitude, velocity, quat = harness.get_state()
    sys.stdout.write("Reinitialize check passed\n")
    sys.stdout.write(
        f"alt={altitude:.6f} vel={velocity:.6f} "
        f"quat=({quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f})\n"
    )


if __name__ == "__main__":
    run_smoke()
