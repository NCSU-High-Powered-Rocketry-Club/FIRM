use crate::constants::command::*;
use crate::data_processor::DataProcessor;
use crate::framed_packet::{FrameError, Framed, FramedPacket};
use crate::utils::{bytes_to_str, parse_bytes_to_f32, parse_bytes_to_many_f32s};
use field_names::FieldNames;
use serde::{Deserialize, Serialize};

#[cfg(feature = "python")]
use pyo3::prelude::*;
#[cfg(feature = "python")]
use pythonize::pythonize;
#[cfg(feature = "wasm")]
use wasm_bindgen::prelude::*;

/// Represents the communication protocol used by the FIRM device.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "python", pyclass(eq, eq_int))]
#[cfg_attr(feature = "wasm", wasm_bindgen)]
pub enum DeviceProtocol {
    USB = 1,
    UART = 2,
    I2C = 3,
    SPI = 4,
}

/// Represents the information of the FIRM device.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "python", pyclass(get_all, set_all))]
pub struct DeviceInfo {
    pub firmware_version: String, // Max 8 characters
    #[cfg_attr(feature = "wasm", serde(serialize_with = "serialize_u64_as_string"))]
    // We need this because JS can't handle u64
    pub id: u64,
}

/// Represents the calibration values for the FIRM device. All matrices
/// are stored in row-major order.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "python", pyclass(get_all, set_all))]
pub struct CalibrationValues {
    pub imu_accelerometer_offsets: [f32; NUMBER_OF_CALIBRATION_OFFSETS],
    pub imu_accelerometer_scale_matrix: [f32; NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS],
    pub imu_gyroscope_offsets: [f32; NUMBER_OF_CALIBRATION_OFFSETS],
    pub imu_gyroscope_scale_matrix: [f32; NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS],
    pub magnetometer_offsets: [f32; NUMBER_OF_CALIBRATION_OFFSETS],
    pub magnetometer_scale_matrix: [f32; NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS],
}

/// Serializes a u64 as a string for WASM compatibility. JS gets unhappy with
/// large integers, such as the device ID, so we serialize it as a string.
#[cfg(feature = "wasm")]
fn serialize_u64_as_string<S>(value: &u64, serializer: S) -> Result<S::Ok, S::Error>
where
    S: serde::Serializer,
{
    serializer.serialize_str(&value.to_string())
}

/// Represents the configuration settings of the FIRM device.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "python", pyclass(get_all, set_all))]
pub struct DeviceConfig {
    pub name: String, // Max 32 characters
    pub frequency: u16,
    pub protocol: DeviceProtocol,
}

/// Represents decoded raw telemetry fields from the wire payload.
///
/// This type intentionally excludes derived fields and is intended for
/// internal processing.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FIRMData {
    pub timestamp_seconds: f64,

    pub temperature_celsius: f32,
    pub pressure_pascals: f32,

    pub raw_acceleration_x_gs: f32,
    pub raw_acceleration_y_gs: f32,
    pub raw_acceleration_z_gs: f32,

    pub raw_angular_rate_x_deg_per_s: f32,
    pub raw_angular_rate_y_deg_per_s: f32,
    pub raw_angular_rate_z_deg_per_s: f32,

    pub magnetic_field_x_microteslas: f32,
    pub magnetic_field_y_microteslas: f32,
    pub magnetic_field_z_microteslas: f32,

    pub est_position_z_meters: f32,

    pub est_velocity_z_meters_per_s: f32,

    pub est_quaternion_w: f32,
    pub est_quaternion_x: f32,
    pub est_quaternion_y: f32,
    pub est_quaternion_z: f32,
}

/// Represents processed telemetry with derived fields included.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, FieldNames)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(name = "FIRMDataPacket", get_all, freelist = 20, frozen)
)]
pub struct ProcessedFIRMData {
    pub timestamp_seconds: f64,

    pub temperature_celsius: f32,
    pub pressure_pascals: f32,

    pub raw_acceleration_x_gs: f32,
    pub raw_acceleration_y_gs: f32,
    pub raw_acceleration_z_gs: f32,

    pub raw_rotated_acceleration_x_gs: f32,
    pub raw_rotated_acceleration_y_gs: f32,
    pub raw_rotated_acceleration_z_gs: f32,
    pub est_tilt_angle_degrees: f32,

    pub raw_angular_rate_x_deg_per_s: f32,
    pub raw_angular_rate_y_deg_per_s: f32,
    pub raw_angular_rate_z_deg_per_s: f32,

    pub magnetic_field_x_microteslas: f32,
    pub magnetic_field_y_microteslas: f32,
    pub magnetic_field_z_microteslas: f32,

    pub est_position_z_meters: f32,

    pub est_velocity_z_meters_per_s: f32,
    pub est_mach_number: f32,

    pub est_quaternion_w: f32,
    pub est_quaternion_x: f32,
    pub est_quaternion_y: f32,
    pub est_quaternion_z: f32,
}

impl DataProcessor {
    pub fn process_firm_data(&mut self, firm_data: &FIRMData) -> ProcessedFIRMData {
        let (
            raw_rotated_acceleration_x_gs,
            raw_rotated_acceleration_y_gs,
            raw_rotated_acceleration_z_gs,
        ) = self.derive_rotated_raw_acceleration(
            firm_data.raw_acceleration_x_gs,
            firm_data.raw_acceleration_y_gs,
            firm_data.raw_acceleration_z_gs,
            firm_data.est_quaternion_w,
            firm_data.est_quaternion_x,
            firm_data.est_quaternion_y,
            firm_data.est_quaternion_z,
        );
        let est_tilt_angle_degrees = self.derive_tilt_angle_degrees(
            firm_data.raw_acceleration_x_gs,
            firm_data.raw_acceleration_y_gs,
            firm_data.raw_acceleration_z_gs,
            firm_data.est_quaternion_w,
            firm_data.est_quaternion_x,
            firm_data.est_quaternion_y,
            firm_data.est_quaternion_z,
        );
        let est_mach_number = self.derive_mach_number(
            firm_data.est_velocity_z_meters_per_s,
            firm_data.temperature_celsius,
        );

        ProcessedFIRMData {
            timestamp_seconds: firm_data.timestamp_seconds,
            temperature_celsius: firm_data.temperature_celsius,
            pressure_pascals: firm_data.pressure_pascals,
            raw_acceleration_x_gs: firm_data.raw_acceleration_x_gs,
            raw_acceleration_y_gs: firm_data.raw_acceleration_y_gs,
            raw_acceleration_z_gs: firm_data.raw_acceleration_z_gs,
            raw_rotated_acceleration_x_gs,
            raw_rotated_acceleration_y_gs,
            raw_rotated_acceleration_z_gs,
            est_tilt_angle_degrees,
            raw_angular_rate_x_deg_per_s: firm_data.raw_angular_rate_x_deg_per_s,
            raw_angular_rate_y_deg_per_s: firm_data.raw_angular_rate_y_deg_per_s,
            raw_angular_rate_z_deg_per_s: firm_data.raw_angular_rate_z_deg_per_s,
            magnetic_field_x_microteslas: firm_data.magnetic_field_x_microteslas,
            magnetic_field_y_microteslas: firm_data.magnetic_field_y_microteslas,
            magnetic_field_z_microteslas: firm_data.magnetic_field_z_microteslas,
            est_position_z_meters: firm_data.est_position_z_meters,
            est_velocity_z_meters_per_s: firm_data.est_velocity_z_meters_per_s,
            est_mach_number,
            est_quaternion_w: firm_data.est_quaternion_w,
            est_quaternion_x: firm_data.est_quaternion_x,
            est_quaternion_y: firm_data.est_quaternion_y,
            est_quaternion_z: firm_data.est_quaternion_z,
        }
    }
}

#[cfg(feature = "python")]
impl ProcessedFIRMData {
    #[allow(clippy::too_many_arguments)]
    fn from_base_fields(
        timestamp_seconds: f64,
        temperature_celsius: f32,
        pressure_pascals: f32,
        raw_acceleration_x_gs: f32,
        raw_acceleration_y_gs: f32,
        raw_acceleration_z_gs: f32,
        raw_angular_rate_x_deg_per_s: f32,
        raw_angular_rate_y_deg_per_s: f32,
        raw_angular_rate_z_deg_per_s: f32,
        magnetic_field_x_microteslas: f32,
        magnetic_field_y_microteslas: f32,
        magnetic_field_z_microteslas: f32,
        est_position_z_meters: f32,
        est_velocity_z_meters_per_s: f32,
        est_quaternion_w: f32,
        est_quaternion_x: f32,
        est_quaternion_y: f32,
        est_quaternion_z: f32,
    ) -> Self {
        let mut processor = DataProcessor::default();
        let firm_data = FIRMData {
            timestamp_seconds,
            temperature_celsius,
            pressure_pascals,
            raw_acceleration_x_gs,
            raw_acceleration_y_gs,
            raw_acceleration_z_gs,
            raw_angular_rate_x_deg_per_s,
            raw_angular_rate_y_deg_per_s,
            raw_angular_rate_z_deg_per_s,
            magnetic_field_x_microteslas,
            magnetic_field_y_microteslas,
            magnetic_field_z_microteslas,
            est_position_z_meters,
            est_velocity_z_meters_per_s,
            est_quaternion_w,
            est_quaternion_x,
            est_quaternion_y,
            est_quaternion_z,
        };
        processor.process_firm_data(&firm_data)
    }
}

#[cfg(feature = "python")]
#[pymethods]
impl ProcessedFIRMData {
    #[classattr]
    fn __struct_fields__() -> Vec<&'static str> {
        ProcessedFIRMData::FIELDS.to_vec()
    }

    #[new]
    #[allow(clippy::too_many_arguments)]
    fn new(
        timestamp_seconds: f64,
        temperature_celsius: f32,
        pressure_pascals: f32,
        raw_acceleration_x_gs: f32,
        raw_acceleration_y_gs: f32,
        raw_acceleration_z_gs: f32,
        raw_angular_rate_x_deg_per_s: f32,
        raw_angular_rate_y_deg_per_s: f32,
        raw_angular_rate_z_deg_per_s: f32,
        magnetic_field_x_microteslas: f32,
        magnetic_field_y_microteslas: f32,
        magnetic_field_z_microteslas: f32,
        est_position_z_meters: f32,
        est_velocity_z_meters_per_s: f32,
        est_quaternion_w: f32,
        est_quaternion_x: f32,
        est_quaternion_y: f32,
        est_quaternion_z: f32,
    ) -> Self {
        Self::from_base_fields(
            timestamp_seconds,
            temperature_celsius,
            pressure_pascals,
            raw_acceleration_x_gs,
            raw_acceleration_y_gs,
            raw_acceleration_z_gs,
            raw_angular_rate_x_deg_per_s,
            raw_angular_rate_y_deg_per_s,
            raw_angular_rate_z_deg_per_s,
            magnetic_field_x_microteslas,
            magnetic_field_y_microteslas,
            magnetic_field_z_microteslas,
            est_position_z_meters,
            est_velocity_z_meters_per_s,
            est_quaternion_w,
            est_quaternion_x,
            est_quaternion_y,
            est_quaternion_z,
        )
    }

    #[staticmethod]
    fn default_zero() -> Self {
        Self::from_base_fields(
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            1.0, // Identity quaternion
            0.0, 0.0, 0.0,
        )
    }

    fn as_dict<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        pythonize(py, self).map_err(|e| {
            use pyo3::exceptions::PyValueError;
            PyValueError::new_err(format!("Failed to serialize packet: {}", e))
        })
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum FIRMResponse {
    GetDeviceInfo(DeviceInfo),
    GetDeviceConfig(DeviceConfig),
    SetDeviceConfig(bool),
    SetMagnetometerCalibration(bool),
    SetIMUCalibration(bool),
    GetCalibration(CalibrationValues),
    Mock(bool),
    Cancel(bool),
    Error(String),
}

/// Wire-level framed data packet.
///
/// This stores both the raw framed bytes and the decoded telemetry.
#[derive(Debug, Clone, PartialEq)]
pub struct FIRMDataPacket {
    frame: FramedPacket,
    data: FIRMData,
}

impl FIRMDataPacket {
    pub fn data(&self) -> &FIRMData {
        &self.data
    }
}

impl Framed for FIRMDataPacket {
    fn frame(&self) -> &FramedPacket {
        &self.frame
    }

    fn from_bytes(bytes: &[u8]) -> Result<Self, FrameError> {
        let frame = FramedPacket::from_bytes(bytes)?;
        Ok(Self {
            data: FIRMData::from_bytes(frame.payload()),
            frame,
        })
    }
}

impl FIRMData {
    /// Constructs a `FIRMData` from a raw payload byte slice.
    pub fn from_bytes(bytes: &[u8]) -> Self {
        let mut idx = 0;

        let timestamp_seconds: f64 = f64::from_le_bytes([
            bytes[idx],
            bytes[idx + 1],
            bytes[idx + 2],
            bytes[idx + 3],
            bytes[idx + 4],
            bytes[idx + 5],
            bytes[idx + 6],
            bytes[idx + 7],
        ]);
        idx += 8;

        let temperature_celsius: f32 = parse_bytes_to_f32(bytes, &mut idx);
        let pressure_pascals: f32 = parse_bytes_to_f32(bytes, &mut idx);

        let raw_acceleration_x_gs: f32 = parse_bytes_to_f32(bytes, &mut idx);
        let raw_acceleration_y_gs: f32 = parse_bytes_to_f32(bytes, &mut idx);
        let raw_acceleration_z_gs: f32 = parse_bytes_to_f32(bytes, &mut idx);

        let raw_angular_rate_x_deg_per_s: f32 = parse_bytes_to_f32(bytes, &mut idx);
        let raw_angular_rate_y_deg_per_s: f32 = parse_bytes_to_f32(bytes, &mut idx);
        let raw_angular_rate_z_deg_per_s: f32 = parse_bytes_to_f32(bytes, &mut idx);

        let magnetic_field_x_microteslas: f32 = parse_bytes_to_f32(bytes, &mut idx);
        let magnetic_field_y_microteslas: f32 = parse_bytes_to_f32(bytes, &mut idx);
        let magnetic_field_z_microteslas: f32 = parse_bytes_to_f32(bytes, &mut idx);

        let est_position_z_meters: f32 = parse_bytes_to_f32(bytes, &mut idx);

        let est_velocity_z_meters_per_s: f32 = parse_bytes_to_f32(bytes, &mut idx);

        let est_quaternion_w: f32 = parse_bytes_to_f32(bytes, &mut idx);
        let est_quaternion_x: f32 = parse_bytes_to_f32(bytes, &mut idx);
        let est_quaternion_y: f32 = parse_bytes_to_f32(bytes, &mut idx);
        let est_quaternion_z: f32 = parse_bytes_to_f32(bytes, &mut idx);

        Self {
            timestamp_seconds,
            temperature_celsius,
            pressure_pascals,
            raw_acceleration_x_gs,
            raw_acceleration_y_gs,
            raw_acceleration_z_gs,
            raw_angular_rate_x_deg_per_s,
            raw_angular_rate_y_deg_per_s,
            raw_angular_rate_z_deg_per_s,
            magnetic_field_x_microteslas,
            magnetic_field_y_microteslas,
            magnetic_field_z_microteslas,
            est_position_z_meters,
            est_velocity_z_meters_per_s,
            est_quaternion_w,
            est_quaternion_x,
            est_quaternion_y,
            est_quaternion_z,
        }
    }
}

/// Wire-level framed response packet.
///
/// The response marker is stored in the identifier u16; the payload is marker-free.
#[derive(Debug, Clone, PartialEq)]
pub struct FIRMResponsePacket {
    frame: FramedPacket,
    command_type: FIRMCommand,
    response: FIRMResponse,
}

impl FIRMResponsePacket {
    pub fn command_type(&self) -> FIRMCommand {
        self.command_type
    }

    pub fn response(&self) -> &FIRMResponse {
        &self.response
    }
}

impl Framed for FIRMResponsePacket {
    fn frame(&self) -> &FramedPacket {
        &self.frame
    }

    fn from_bytes(bytes: &[u8]) -> Result<Self, FrameError> {
        let frame = FramedPacket::from_bytes(bytes)?;
        let command_type = FIRMCommand::from_u16(frame.identifier())?;
        let response = FIRMResponse::from_command_and_bytes(command_type, frame.payload());

        Ok(Self {
            frame,
            command_type,
            response,
        })
    }
}

impl FIRMResponse {
    /// Constructs a decoded `FIRMResponse` from a command and raw payload bytes.
    pub fn from_command_and_bytes(command: FIRMCommand, data: &[u8]) -> Self {
        match command {
            FIRMCommand::GetDeviceInfo => {
                // [ID (8 bytes)][FIRMWARE_VERSION (8 bytes)][PADDING ...]
                let id_bytes = &data[0..DEVICE_ID_LENGTH];
                let firmware_version_bytes =
                    &data[DEVICE_ID_LENGTH..DEVICE_ID_LENGTH + FIRMWARE_VERSION_LENGTH];
                let id = u64::from_le_bytes(id_bytes.try_into().unwrap());
                let firmware_version = bytes_to_str(firmware_version_bytes);

                let info = DeviceInfo {
                    id,
                    firmware_version,
                };
                FIRMResponse::GetDeviceInfo(info)
            }
            FIRMCommand::GetDeviceConfig => {
                // [NAME (32 bytes)][FREQUENCY (2 bytes)][PROTOCOL (1 byte)]
                let name_bytes: [u8; DEVICE_NAME_LENGTH] =
                    data[0..DEVICE_NAME_LENGTH].try_into().unwrap();
                let name = bytes_to_str(&name_bytes);
                let frequency = u16::from_le_bytes(
                    data[DEVICE_NAME_LENGTH..DEVICE_NAME_LENGTH + FREQUENCY_LENGTH]
                        .try_into()
                        .unwrap(),
                );
                let protocol_byte = data[DEVICE_NAME_LENGTH + FREQUENCY_LENGTH];
                let protocol = match protocol_byte {
                    1 => DeviceProtocol::USB,
                    2 => DeviceProtocol::UART,
                    3 => DeviceProtocol::I2C,
                    4 => DeviceProtocol::SPI,
                    _ => DeviceProtocol::USB, // Fallback for invalid values
                };

                let config = DeviceConfig {
                    frequency,
                    protocol,
                    name,
                };

                FIRMResponse::GetDeviceConfig(config)
            }
            FIRMCommand::SetDeviceConfig => {
                let success = data.first() == Some(&1);
                FIRMResponse::SetDeviceConfig(success)
            }
            FIRMCommand::Mock => {
                let success = data.first() == Some(&1);
                FIRMResponse::Mock(success)
            }
            FIRMCommand::Cancel => {
                let acknowledgement = data.first() == Some(&1);
                FIRMResponse::Cancel(acknowledgement)
            }
            FIRMCommand::SetMagnetometerCalibration => {
                let success = data.first() == Some(&1);
                FIRMResponse::SetMagnetometerCalibration(success)
            }
            FIRMCommand::SetIMUCalibration => {
                let success = data.first() == Some(&1);
                FIRMResponse::SetIMUCalibration(success)
            }
            FIRMCommand::GetCalibration => {
                let mut idx = 0;
                let imu_accelerometer_offsets =
                    parse_bytes_to_many_f32s(data, NUMBER_OF_CALIBRATION_OFFSETS, &mut idx);
                let imu_accelerometer_scale_matrix = parse_bytes_to_many_f32s(
                    data,
                    NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS,
                    &mut idx,
                );
                let imu_gyroscope_offsets =
                    parse_bytes_to_many_f32s(data, NUMBER_OF_CALIBRATION_OFFSETS, &mut idx);
                let imu_gyroscope_scale_matrix = parse_bytes_to_many_f32s(
                    data,
                    NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS,
                    &mut idx,
                );
                let magnetometer_offsets =
                    parse_bytes_to_many_f32s(data, NUMBER_OF_CALIBRATION_OFFSETS, &mut idx);
                let magnetometer_scale_matrix = parse_bytes_to_many_f32s(
                    data,
                    NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS,
                    &mut idx,
                );
                FIRMResponse::GetCalibration(CalibrationValues {
                    imu_accelerometer_offsets: imu_accelerometer_offsets.try_into().unwrap(),
                    imu_accelerometer_scale_matrix: imu_accelerometer_scale_matrix
                        .try_into()
                        .unwrap(),
                    imu_gyroscope_offsets: imu_gyroscope_offsets.try_into().unwrap(),
                    imu_gyroscope_scale_matrix: imu_gyroscope_scale_matrix.try_into().unwrap(),
                    magnetometer_offsets: magnetometer_offsets.try_into().unwrap(),
                    magnetometer_scale_matrix: magnetometer_scale_matrix.try_into().unwrap(),
                })
            }
            // Reboot currently has no decoded response type.
            FIRMCommand::Reboot => {
                FIRMResponse::Error("No decoded response for Reboot".to_string())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        DeviceConfig, DeviceInfo, DeviceProtocol, FIRMData, FIRMResponse, FIRMResponsePacket,
    };
    use crate::constants::command::{
        DEVICE_ID_LENGTH, DEVICE_NAME_LENGTH, FIRMCommand, FIRMWARE_VERSION_LENGTH,
        FREQUENCY_LENGTH,
    };
    use crate::constants::packet::PacketHeader;
    use crate::framed_packet::{FrameError, Framed, FramedPacket};
    use crate::utils::str_to_bytes;

    fn resp_set_device_config(v: bool) -> FIRMResponse {
        FIRMResponse::SetDeviceConfig(v)
    }

    fn resp_mock(v: bool) -> FIRMResponse {
        FIRMResponse::Mock(v)
    }

    fn resp_cancel(v: bool) -> FIRMResponse {
        FIRMResponse::Cancel(v)
    }

    fn build_response_packet(
        identifier: u16,
        payload: &[u8],
    ) -> Result<FIRMResponsePacket, FrameError> {
        let bytes =
            FramedPacket::new(PacketHeader::Response, identifier, payload.to_vec()).to_bytes();
        FIRMResponsePacket::from_bytes(&bytes)
    }

    #[test]
    fn test_firm_data_packet_from_bytes() {
        let mut payload = [0u8; 120];
        let timestamp = 42.0f64;
        let temperature = 25.0f32;
        let pressure = 101325.0f32;
        payload[0..8].copy_from_slice(&timestamp.to_le_bytes());
        payload[8..12].copy_from_slice(&temperature.to_le_bytes());
        payload[12..16].copy_from_slice(&pressure.to_le_bytes());

        let pkt = FIRMData::from_bytes(&payload);
        assert_eq!(pkt.timestamp_seconds, timestamp);
        assert_eq!(pkt.temperature_celsius, temperature);
        assert_eq!(pkt.pressure_pascals, pressure);
    }

    #[test]
    fn test_firm_response_packet_from_bytes_get_device_info() {
        let mut payload = [0u8; DEVICE_ID_LENGTH + FIRMWARE_VERSION_LENGTH];

        let id = 0x1122334455667788u64;
        payload[0..DEVICE_ID_LENGTH].copy_from_slice(&id.to_le_bytes());

        let fw_bytes = str_to_bytes::<FIRMWARE_VERSION_LENGTH>("v1.2.3");
        payload[DEVICE_ID_LENGTH..DEVICE_ID_LENGTH + FIRMWARE_VERSION_LENGTH]
            .copy_from_slice(&fw_bytes);

        let pkt = build_response_packet(FIRMCommand::GetDeviceInfo as u16, &payload).unwrap();
        assert_eq!(
            pkt.response(),
            &FIRMResponse::GetDeviceInfo(DeviceInfo {
                firmware_version: "v1.2.3".to_string(),
                id,
            })
        );
        assert_eq!(pkt.command_type(), FIRMCommand::GetDeviceInfo);
    }

    #[test]
    fn test_firm_response_packet_from_bytes_get_device_config() {
        let mut payload = [0u8; DEVICE_NAME_LENGTH + FREQUENCY_LENGTH + 1];

        let name_bytes = str_to_bytes::<DEVICE_NAME_LENGTH>("MyDevice");
        payload[0..DEVICE_NAME_LENGTH].copy_from_slice(&name_bytes);

        let frequency: u16 = 50;
        payload[DEVICE_NAME_LENGTH..DEVICE_NAME_LENGTH + FREQUENCY_LENGTH]
            .copy_from_slice(&frequency.to_le_bytes());

        payload[DEVICE_NAME_LENGTH + FREQUENCY_LENGTH] = 0x03;

        let pkt = build_response_packet(FIRMCommand::GetDeviceConfig as u16, &payload).unwrap();
        assert_eq!(
            pkt.response(),
            &FIRMResponse::GetDeviceConfig(DeviceConfig {
                name: "MyDevice".to_string(),
                frequency,
                protocol: DeviceProtocol::I2C,
            })
        );
        assert_eq!(pkt.command_type(), FIRMCommand::GetDeviceConfig);
    }

    #[test]
    fn test_firm_response_packet_from_bytes_set_device_config() {
        let cases: &[(u16, FIRMCommand, fn(bool) -> FIRMResponse)] = &[
            (
                FIRMCommand::SetDeviceConfig as u16,
                FIRMCommand::SetDeviceConfig,
                resp_set_device_config,
            ),
            (FIRMCommand::Mock as u16, FIRMCommand::Mock, resp_mock),
            (FIRMCommand::Cancel as u16, FIRMCommand::Cancel, resp_cancel),
        ];

        for (identifier, expected_command_type, mk_response) in cases {
            let pkt = build_response_packet(*identifier, &[1u8]).unwrap();
            assert_eq!(pkt.response(), &mk_response(true));
            assert_eq!(pkt.command_type(), *expected_command_type);
        }
    }

    #[test]
    fn test_firm_response_packet_from_bytes_unknown_identifier() {
        let payload = [0u8];
        let err = build_response_packet(0x00AB, &payload).unwrap_err();
        assert_eq!(err, FrameError::UnknownIdentifier(0x00AB));
    }
}
