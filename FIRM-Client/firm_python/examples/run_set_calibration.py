import argparse
from firm_client import FIRMClient


# uv run .\firm_python\examples\run_set_calibration.py COM12

# Example calibration values
# Offsets: [x, y, z]
ACCEL_OFFSETS = (1.0, 0.0, 0.0)
GYRO_OFFSETS = (0.0, 3.0, 0.0)
MAG_OFFSETS = (0.5, -0.02, 0.005)

# Scale matrices: 3x3 row-major
ACCEL_IDENTITY_SCALE = (
    3.0,
    0.0,
    2.0,
    0.0,
    1.0,
    0.0,
    8.0,
    0.0,
    1.0,
)

GYRO_IDENTITY_SCALE = (
    1.0,
    5.0,
    3.0,
    0.0,
    1.0,
    0.0,
    6.0,
    0.0,
    9.0,
)

MAG_IDENTITY_SCALE = (
    10.0,
    0.0,
    0.0,
    0.0,
    -1.0,
    0.0,
    2.0,
    0.0,
    -3.0,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Set FIRM calibration constants")
    parser.add_argument("port", help='Serial port (e.g. "COM8" or "/dev/ttyACM0")')
    parser.add_argument(
        "--baud",
        type=int,
        default=2_000_000,
        help="Serial baud rate (default: 2000000)",
    )
    args = parser.parse_args()

    with FIRMClient(args.port, args.baud) as client:
        print("Setting magnetometer calibration...")
        ok = client.set_magnetometer_calibration(
            MAG_OFFSETS,
            MAG_IDENTITY_SCALE,
        )
        print(f"Magnetometer calibration {'OK' if ok else 'FAILED'}")

        print("Setting IMU calibration...")
        ok = client.set_imu_calibration(
            ACCEL_OFFSETS,
            ACCEL_IDENTITY_SCALE,
            GYRO_OFFSETS,
            GYRO_IDENTITY_SCALE,
        )
        print(f"IMU calibration {'OK' if ok else 'FAILED'}")

        print("Reading calibration back from device...")
        cal = client.get_calibration(timeout_seconds=5.0)
        if cal is None:
            print("No calibration response (timeout). Try a larger timeout_seconds.")
        else:
            print("imu_accelerometer_offsets:", cal.imu_accelerometer_offsets)
            print("imu_accelerometer_scale_matrix:", cal.imu_accelerometer_scale_matrix)
            print("imu_gyroscope_offsets:", cal.imu_gyroscope_offsets)
            print("imu_gyroscope_scale_matrix:", cal.imu_gyroscope_scale_matrix)
            print("magnetometer_offsets:", cal.magnetometer_offsets)
            print("magnetometer_scale_matrix:", cal.magnetometer_scale_matrix)


if __name__ == "__main__":
    main()
