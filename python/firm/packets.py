"""Defines the data packet structures used in FIRM."""

import msgspec


class FirmPacket(msgspec.Struct):
    """Base class for FIRM data packets.

    Arguments:
        timestamp_secs (float): Timestamp in seconds since FIRM was powered on.
    """

    timestamp_secs: float


class IMUPacket(FirmPacket):
    """Inertial Measurement Unit (IMU) data packet.

    Arguments:
        acc_x_gs (float): Acceleration in the X direction in g's.
        acc_y_gs (float): Acceleration in the Y direction in g's.
        acc_z_gs (float): Acceleration in the Z direction in g's.
        gyro_x_rad_s (float): Angular rate around the X axis in radians per second.
        gyro_y_rad_s (float): Angular rate around the Y axis in radians per second.
        gyro_z_rad_s (float): Angular rate around the Z axis in radians per second.
    """

    acc_x_gs: float
    acc_y_gs: float
    acc_z_gs: float
    gyro_x_rad_s: float
    gyro_y_rad_s: float
    gyro_z_rad_s: float


class BarometerPacket(FirmPacket):
    """Barometer data packet.

    Arguments:
        pressure_pascals (float): Atmospheric pressure in Pascals.
        temperature_celsius (float): Temperature in degrees Celsius.
    """

    pressure_pascals: float
    temperature_celsius: float


class MagnetometerPacket(FirmPacket):
    """Magnetometer data packet.

    Arguments:
        mag_x_ut (float): Magnetic field in the X direction in microteslas.
        mag_y_ut (float): Magnetic field in the Y direction in microteslas.
        mag_z_ut (float): Magnetic field in the Z direction in microteslas.
    """

    mag_x_ut: float
    mag_y_ut: float
    mag_z_ut: float
