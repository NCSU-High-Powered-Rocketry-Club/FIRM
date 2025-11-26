"""Defines the unified FIRM data packet structure."""

from msgspec import Struct


class FIRMPacket(Struct):
    """Full FIRM data packet containing all sensor fields.

    Arguments:
        timestamp_seconds (float): Timestamp in seconds since FIRM was powered on.
        accel_x_meters_per_s2 (float): Acceleration in the X direction in meters per second squared.
        accel_y_meters_per_s2 (float): Acceleration in the Y direction in meters per second squared.
        accel_z_meters_per_s2 (float): Acceleration in the Z direction in meters per second squared.
        gyro_x_radians_per_s (float): Angular rate around the X axis in radians per second.
        gyro_y_radians_per_s (float): Angular rate around the Y axis in radians per second.
        gyro_z_radians_per_s (float): Angular rate around the Z axis in radians per second.
        pressure_pascals (float): Atmospheric pressure in Pascals.
        temperature_celsius (float): Temperature in degrees Celsius.
        mag_x_microteslas (float): Magnetic field in the X direction in microteslas.
        mag_y_microteslas (float): Magnetic field in the Y direction in microteslas.
        mag_z_microteslas (float): Magnetic field in the Z direction in microteslas.
        pressure_altitude_meters (float): Calculated altitude from pressure in meters.
    """

    timestamp_seconds: float

    accel_x_meters_per_s2: float
    accel_y_meters_per_s2: float
    accel_z_meters_per_s2: float

    gyro_x_radians_per_s: float
    gyro_y_radians_per_s: float
    gyro_z_radians_per_s: float

    pressure_pascals: float
    temperature_celsius: float

    mag_x_microteslas: float
    mag_y_microteslas: float
    mag_z_microteslas: float

    pressure_altitude_meters: float
