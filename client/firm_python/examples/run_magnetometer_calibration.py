import firm_client

client = firm_client.FIRMClient("COM9")
client.start()


def print_mag_calibration(label: str) -> None:
    cal = client.get_calibration(timeout_seconds=2.0)
    if cal is None:
        print(f"{label}: <no calibration response>")
        return

    print(f"{label}:")
    print(f"  mag offsets: {cal.magnetometer_offsets}")
    print(f"  mag matrix:  {cal.magnetometer_scale_matrix}")


print_mag_calibration("Before calibration")

print("Please rotate the device in all directions...")

# This line will block for 30 seconds
result = client.run_and_apply_magnetometer_calibration(
    collection_duration_seconds=30.0, apply_timeout_seconds=1.0
)

if result is True:
    print("Calibration Success! Applied to device.")
elif result is False:
    print("Calibration calculated, but device rejected the update.")
elif result is None:
    print("Calibration Failed: Not enough data points.")
else:
    print("Error occurred.")

print_mag_calibration("After calibration")

client.stop()
