import inspect
import math

from firm_client import FIRMDataPacket
import pytest


def test_firm_data_packet_constructor() -> None:
    packet = FIRMDataPacket(
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
        6.0,
        7.0,
        8.0,
        9.0,
        10.0,
        11.0,
        12.0,
        15.0,
        18.0,
        19.0,
        0.0,
        0.0,
        0.0,
    )

    assert packet.timestamp_seconds == 1.0
    assert packet.temperature_celsius == 2.0
    assert packet.pressure_pascals == 3.0
    assert packet.raw_acceleration_x_gs == 4.0
    assert packet.raw_acceleration_y_gs == 5.0
    assert packet.raw_acceleration_z_gs == 6.0
    assert packet.raw_angular_rate_x_deg_per_s == 7.0
    assert packet.raw_angular_rate_y_deg_per_s == 8.0
    assert packet.raw_angular_rate_z_deg_per_s == 9.0
    assert packet.magnetic_field_x_microteslas == 10.0
    assert packet.magnetic_field_y_microteslas == 11.0
    assert packet.magnetic_field_z_microteslas == 12.0
    assert packet.est_position_z_meters == 15.0
    assert packet.est_velocity_z_meters_per_s == 18.0
    assert packet.est_quaternion_w == 19.0
    assert packet.est_quaternion_x == 0.0
    assert packet.est_quaternion_y == 0.0
    assert packet.est_quaternion_z == 0.0

    expected_rotated_x = 4.0 * math.cos(math.radians(45.0)) - 5.0 * math.sin(
        math.radians(45.0)
    )
    expected_rotated_y = 4.0 * math.sin(math.radians(45.0)) + 5.0 * math.cos(
        math.radians(45.0)
    )
    expected_rotated_z = 6.0

    assert packet.raw_rotated_acceleration_x_gs == pytest.approx(
        expected_rotated_x, rel=1e-6, abs=1e-6
    )
    assert packet.raw_rotated_acceleration_y_gs == pytest.approx(
        expected_rotated_y, rel=1e-6, abs=1e-6
    )
    assert packet.raw_rotated_acceleration_z_gs == pytest.approx(
        expected_rotated_z, rel=1e-6, abs=1e-6
    )

    # Tilt is quaternion-based after axis latching; this synthetic sample latches to +Y
    # and identity quaternion maps +Y to world +Y, i.e. 90 deg from world +Z.
    expected_tilt = 90.0
    assert packet.est_tilt_angle_degrees == pytest.approx(
        expected_tilt, rel=1e-6, abs=1e-6
    )

    temperature_kelvin = 2.0 + 273.15
    speed_of_sound = math.sqrt(1.4 * 287.05 * temperature_kelvin)
    expected_mach = abs(18.0) / speed_of_sound
    assert packet.est_mach_number == pytest.approx(expected_mach, rel=1e-6, abs=1e-6)


def test_firm_data_packet_default_zero() -> None:
    firm_data_packet = FIRMDataPacket.default_zero()

    assert firm_data_packet.timestamp_seconds == 0.0
    assert firm_data_packet.temperature_celsius == 0.0
    assert firm_data_packet.pressure_pascals == 0.0
    assert firm_data_packet.raw_acceleration_x_gs == 0.0
    assert firm_data_packet.raw_acceleration_y_gs == 0.0
    assert firm_data_packet.raw_acceleration_z_gs == 0.0
    assert firm_data_packet.raw_angular_rate_x_deg_per_s == 0.0
    assert firm_data_packet.raw_angular_rate_y_deg_per_s == 0.0
    assert firm_data_packet.raw_angular_rate_z_deg_per_s == 0.0
    assert firm_data_packet.magnetic_field_x_microteslas == 0.0
    assert firm_data_packet.magnetic_field_y_microteslas == 0.0
    assert firm_data_packet.magnetic_field_z_microteslas == 0.0
    assert firm_data_packet.est_position_z_meters == 0.0
    assert firm_data_packet.est_velocity_z_meters_per_s == 0.0
    assert firm_data_packet.est_quaternion_w == 1.0
    assert firm_data_packet.est_quaternion_x == 0.0
    assert firm_data_packet.est_quaternion_y == 0.0
    assert firm_data_packet.est_quaternion_z == 0.0
    assert firm_data_packet.raw_rotated_acceleration_x_gs == 0.0
    assert firm_data_packet.raw_rotated_acceleration_y_gs == 0.0
    assert firm_data_packet.raw_rotated_acceleration_z_gs == 0.0
    assert firm_data_packet.est_tilt_angle_degrees == 0.0
    assert firm_data_packet.est_mach_number == 0.0


def test_firm_data_packet_struct_fields() -> None:
    fields = FIRMDataPacket.__struct_fields__

    assert isinstance(fields, list)

    sig = inspect.signature(FIRMDataPacket)
    constructor_params = list(sig.parameters.keys())

    assert set(constructor_params).issubset(set(fields))
    assert "raw_rotated_acceleration_x_gs" in fields
    assert "raw_rotated_acceleration_y_gs" in fields
    assert "raw_rotated_acceleration_z_gs" in fields
    assert "est_tilt_angle_degrees" in fields
    assert "est_mach_number" in fields


def test_firm_data_packet_as_dict() -> None:
    packet = FIRMDataPacket(
        timestamp_seconds=1.0,
        temperature_celsius=2.0,
        pressure_pascals=3.0,
        raw_acceleration_x_gs=4.0,
        raw_acceleration_y_gs=5.0,
        raw_acceleration_z_gs=6.0,
        raw_angular_rate_x_deg_per_s=7.0,
        raw_angular_rate_y_deg_per_s=8.0,
        raw_angular_rate_z_deg_per_s=9.0,
        magnetic_field_x_microteslas=10.0,
        magnetic_field_y_microteslas=11.0,
        magnetic_field_z_microteslas=12.0,
        est_position_z_meters=15.0,
        est_velocity_z_meters_per_s=18.0,
        est_quaternion_w=19.0,
        est_quaternion_x=0.0,
        est_quaternion_y=0.0,
        est_quaternion_z=0.0,
    )

    data_dict = packet.as_dict()

    assert isinstance(data_dict, dict)

    assert set(data_dict.keys()) == set(FIRMDataPacket.__struct_fields__)

    assert data_dict["timestamp_seconds"] == 1.0
    assert data_dict["temperature_celsius"] == 2.0
    assert data_dict["est_quaternion_z"] == 0.0
    expected_rotated_x = 4.0 * math.cos(math.radians(45.0)) - 5.0 * math.sin(
        math.radians(45.0)
    )
    assert data_dict["raw_rotated_acceleration_x_gs"] == pytest.approx(
        expected_rotated_x, rel=1e-6, abs=1e-6
    )

    # Make sure modifying the dict does not affect the original packet
    data_dict["timestamp_seconds"] = 999.9
    assert packet.timestamp_seconds == 1.0
