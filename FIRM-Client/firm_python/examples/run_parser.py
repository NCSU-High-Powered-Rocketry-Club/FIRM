from firm_client import FIRMClient


def render_packet(packet) -> str:
    return "\n".join(
        [
            f"--- FIRM Packet ({packet.timestamp_seconds:.4f}s) ---",
            f"Env:      {packet.temperature_celsius:.2f}°C | {packet.pressure_pascals:.2f} Pa",
            "",
            f"Raw Accel (G):      x={packet.raw_acceleration_x_gs: >8.4f}, y={packet.raw_acceleration_y_gs: >8.4f}, z={packet.raw_acceleration_z_gs: >8.4f}",
            f"Rot Accel (G):      x={packet.raw_rotated_acceleration_x_gs: >8.4f}, y={packet.raw_rotated_acceleration_y_gs: >8.4f}, z={packet.raw_rotated_acceleration_z_gs: >8.4f}",
            f"Est Tilt (deg):     {packet.est_tilt_angle_degrees: >8.4f}",
            f"Est Mach:           {packet.est_mach_number: >8.4f}",
            f"Raw Gyro (d/s):     x={packet.raw_angular_rate_x_deg_per_s: >8.4f}, y={packet.raw_angular_rate_y_deg_per_s: >8.4f}, z={packet.raw_angular_rate_z_deg_per_s: >8.4f}",
            f"Mag Field (uT):     x={packet.magnetic_field_x_microteslas: >8.4f}, y={packet.magnetic_field_y_microteslas: >8.4f}, z={packet.magnetic_field_z_microteslas: >8.4f}",
            f"Est Pos (m):        z={packet.est_position_z_meters: >8.4f}",
            f"Est Vel (m/s):      z={packet.est_velocity_z_meters_per_s: >8.4f}",
            f"Est Quat:           w={packet.est_quaternion_w: >6.3f}, x={packet.est_quaternion_x: >6.3f}, y={packet.est_quaternion_y: >6.3f}, z={packet.est_quaternion_z: >6.3f}",
            "-------------------------------------------",
        ]
    )


def main() -> None:
    port = "COM14"  # Update as needed (e.g., "COM8" or "/dev/ttyACM0")
    baud_rate = 2_000_000

    with FIRMClient(port, baud_rate, timeout=0.2) as client:
        client.get_data_packets(block=True)  # Clear initial packets
        while client.is_running():
            packets = client.get_data_packets(block=True)
            if not packets:
                continue

            packet = packets[-1]
            print("\x1b[2J\x1b[H", end="")
            print(render_packet(packet), flush=True)


main()
