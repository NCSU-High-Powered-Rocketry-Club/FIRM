export interface FIRMPacket {
  /** Timestamp of the data packet in seconds. */
  timestamp_seconds: number;

  /** Ambient temperature measured in degrees Celsius. */
  temperature_celsius: number;
  /** Atmospheric pressure measured in Pascals. */
  pressure_pascals: number;

  /** Raw accelerometer reading for X-axis in Gs. */
  raw_acceleration_x_gs: number;
  /** Raw accelerometer reading for Y-axis in Gs. */
  raw_acceleration_y_gs: number;
  /** Raw accelerometer reading for Z-axis in Gs. */
  raw_acceleration_z_gs: number;

  /** Raw acceleration X rotated into world frame in Gs. */
  raw_rotated_acceleration_x_gs: number;
  /** Raw acceleration Y rotated into world frame in Gs. */
  raw_rotated_acceleration_y_gs: number;
  /** Raw acceleration Z rotated into world frame in Gs. */
  raw_rotated_acceleration_z_gs: number;

  /** Raw gyroscope reading for X-axis in degrees per second. */
  raw_angular_rate_x_deg_per_s: number;
  /** Raw gyroscope reading for Y-axis in degrees per second. */
  raw_angular_rate_y_deg_per_s: number;
  /** Raw gyroscope reading for Z-axis in degrees per second. */
  raw_angular_rate_z_deg_per_s: number;

  /** Magnetometer reading for X-axis in micro-Teslas. */
  magnetic_field_x_microteslas: number;
  /** Magnetometer reading for Y-axis in micro-Teslas. */
  magnetic_field_y_microteslas: number;
  /** Magnetometer reading for Z-axis in micro-Teslas. */
  magnetic_field_z_microteslas: number;

  /** Estimated position along the Z-axis in meters. */
  est_position_z_meters: number;

  /** Estimated velocity along the Z-axis in meters per second. */
  est_velocity_z_meters_per_s: number;

  /** Estimated Mach number from velocity magnitude and ambient temperature. */
  est_mach_number: number;

  /** Estimated orientation quaternion scalar component (W). */
  est_quaternion_w: number;
  /** Estimated orientation quaternion vector component (X). */
  est_quaternion_x: number;
  /** Estimated orientation quaternion vector component (Y). */
  est_quaternion_y: number;
  /** Estimated orientation quaternion vector component (Z). */
  est_quaternion_z: number;

  /** Total tilt angle from +Z based on raw acceleration in degrees. */
  est_tilt_angle_degrees: number;

}

export enum DeviceProtocol {
  USB = 1,
  UART = 2,
  I2C = 3,
  SPI = 4,
}

export interface DeviceInfo {
  id: string;
  firmware_version: string;
}

export interface DeviceConfig {
  name: string;
  frequency: number;
  protocol: DeviceProtocol;
}

export interface CalibrationValues {
  imu_accelerometer_offsets: [number, number, number];
  imu_accelerometer_scale_matrix: [
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
  ];
  imu_gyroscope_offsets: [number, number, number];
  imu_gyroscope_scale_matrix: [
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
  ];
  magnetometer_offsets: [number, number, number];
  magnetometer_scale_matrix: [
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
  ];
}

export type FIRMResponse =
  | { GetDeviceInfo: DeviceInfo }
  | { GetDeviceConfig: DeviceConfig }
  | { SetDeviceConfig: boolean }
  | { SetMagnetometerCalibration: boolean }
  | { SetIMUCalibration: boolean }
  | { GetCalibration: CalibrationValues }
  | { Mock: boolean }
  | { Cancel: boolean }
  | { Error: string };

