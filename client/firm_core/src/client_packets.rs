use alloc::vec::Vec;

use crate::constants::command::{
    CALIBRATION_OFFSETS_LENGTH, CALIBRATION_SCALE_MATRIX_LENGTH, DEVICE_CONFIG_PAYLOAD_LENGTH,
    DEVICE_NAME_LENGTH, FIRMCommand, IMU_CALIBRATION_PAYLOAD_LENGTH, NUMBER_OF_CALIBRATION_OFFSETS,
    NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS,
};
use crate::constants::log_parsing::FIRMLogPacketType;
use crate::firm_packets::{DeviceConfig, DeviceProtocol};
use crate::utils::str_to_bytes;
use crate::wire_packet::PacketError;

/// A host-to-device command encoded as `[id: u8][payload]`.
pub struct FIRMCommandPacket {
    command_type: FIRMCommand,
    payload: Vec<u8>,
}

impl FIRMCommandPacket {
    pub fn new(command_type: FIRMCommand, payload: Vec<u8>) -> Self {
        Self {
            command_type,
            payload,
        }
    }

    pub fn command_type(&self) -> FIRMCommand {
        self.command_type
    }

    pub fn identifier(&self) -> u8 {
        self.command_type.to_u8()
    }

    pub fn payload(&self) -> &[u8] {
        &self.payload
    }

    pub fn len(&self) -> usize {
        self.payload.len()
    }

    pub fn is_empty(&self) -> bool {
        self.payload.is_empty()
    }

    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(1 + self.payload.len());
        bytes.push(self.identifier());
        bytes.extend_from_slice(&self.payload);
        bytes
    }

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, PacketError> {
        let (&identifier, payload) = bytes.split_first().ok_or(PacketError::TooShort)?;
        let command_type = FIRMCommand::from_u8(identifier)?;
        let expected = 1 + command_type.command_payload_len();
        if bytes.len() != expected {
            return Err(PacketError::LengthMismatch {
                expected,
                got: bytes.len(),
            });
        }
        Ok(Self::new(command_type, payload.to_vec()))
    }

    pub fn build_get_device_info_command() -> Self {
        Self::new(FIRMCommand::GetDeviceInfo, Vec::new())
    }

    pub fn build_get_device_config_command() -> Self {
        Self::new(FIRMCommand::GetDeviceConfig, Vec::new())
    }

    pub fn build_cancel_command() -> Self {
        Self::new(FIRMCommand::Cancel, Vec::new())
    }

    pub fn build_reboot_command() -> Self {
        Self::new(FIRMCommand::Reboot, Vec::new())
    }

    pub fn build_mock_command() -> Self {
        Self::new(FIRMCommand::Mock, Vec::new())
    }

    pub fn build_set_device_config_command(config: DeviceConfig) -> Self {
        let mut payload = Vec::with_capacity(DEVICE_CONFIG_PAYLOAD_LENGTH);
        payload.extend_from_slice(&config.frequency.to_le_bytes());
        payload.extend_from_slice(&str_to_bytes::<DEVICE_NAME_LENGTH>(&config.name));

        // USB is always enabled by the STM32. The selected protocol enables one
        // additional transport, matching DeviceConfig_t's four bool fields.
        payload.push(1);
        payload.push(u8::from(config.protocol == DeviceProtocol::UART));
        payload.push(u8::from(config.protocol == DeviceProtocol::I2C));
        payload.push(u8::from(config.protocol == DeviceProtocol::SPI));

        Self::new(FIRMCommand::SetDeviceConfig, payload)
    }

    pub fn build_set_magnetometer_calibration_command(
        offsets: [f32; NUMBER_OF_CALIBRATION_OFFSETS],
        scale_matrix: [f32; NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS],
    ) -> Self {
        let mut payload =
            Vec::with_capacity(CALIBRATION_OFFSETS_LENGTH + CALIBRATION_SCALE_MATRIX_LENGTH);
        for offset in &offsets {
            payload.extend_from_slice(&offset.to_le_bytes());
        }
        for scale in &scale_matrix {
            payload.extend_from_slice(&scale.to_le_bytes());
        }
        Self::new(FIRMCommand::SetMagnetometerCalibration, payload)
    }

    pub fn build_set_imu_calibration_command(
        accel_offsets: [f32; NUMBER_OF_CALIBRATION_OFFSETS],
        accel_scale_matrix: [f32; NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS],
        gyro_offsets: [f32; NUMBER_OF_CALIBRATION_OFFSETS],
        gyro_scale_matrix: [f32; NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS],
    ) -> Self {
        let mut payload = Vec::with_capacity(IMU_CALIBRATION_PAYLOAD_LENGTH);
        for values in [
            accel_offsets.as_slice(),
            accel_scale_matrix.as_slice(),
            gyro_offsets.as_slice(),
            gyro_scale_matrix.as_slice(),
        ] {
            for value in values {
                payload.extend_from_slice(&value.to_le_bytes());
            }
        }
        Self::new(FIRMCommand::SetIMUCalibration, payload)
    }

    pub fn build_get_calibration_command() -> Self {
        Self::new(FIRMCommand::GetCalibration, Vec::new())
    }
}

/// A mock-log message encoded as `[sensor_id: u8][timestamp + sensor payload]`.
#[derive(Debug, Clone, PartialEq)]
pub struct FIRMLogPacket {
    packet_type: FIRMLogPacketType,
    payload: Vec<u8>,
}

impl FIRMLogPacket {
    pub fn new(packet_type: FIRMLogPacketType, payload: Vec<u8>) -> Self {
        Self {
            packet_type,
            payload,
        }
    }

    pub fn packet_type(&self) -> FIRMLogPacketType {
        self.packet_type
    }

    pub fn identifier(&self) -> u8 {
        self.packet_type.as_u8()
    }

    pub fn payload(&self) -> &[u8] {
        &self.payload
    }

    pub fn len(&self) -> usize {
        self.payload.len()
    }

    pub fn is_empty(&self) -> bool {
        self.payload.is_empty()
    }

    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(1 + self.payload.len());
        bytes.push(self.identifier());
        bytes.extend_from_slice(&self.payload);
        bytes
    }

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, PacketError> {
        let (&identifier, payload) = bytes.split_first().ok_or(PacketError::TooShort)?;
        let packet_type = FIRMLogPacketType::from_u8(identifier)
            .ok_or(PacketError::UnknownIdentifier(identifier))?;
        Ok(Self::new(packet_type, payload.to_vec()))
    }
}

#[cfg(test)]
mod tests {
    use super::{FIRMCommandPacket, FIRMLogPacket};
    use crate::constants::command::{
        DEVICE_CONFIG_PAYLOAD_LENGTH, FIRMCommand, IMU_CALIBRATION_PAYLOAD_LENGTH,
    };
    use crate::constants::log_parsing::FIRMLogPacketType;
    use crate::firm_packets::{DeviceConfig, DeviceProtocol};

    fn f32_from_payload(payload: &[u8], idx: usize) -> f32 {
        let start = idx * 4;
        f32::from_le_bytes(payload[start..start + 4].try_into().unwrap())
    }

    #[test]
    fn zero_payload_commands_are_one_identifier_byte() {
        let cases = [
            FIRMCommandPacket::build_get_device_info_command(),
            FIRMCommandPacket::build_get_device_config_command(),
            FIRMCommandPacket::build_cancel_command(),
            FIRMCommandPacket::build_reboot_command(),
            FIRMCommandPacket::build_mock_command(),
        ];
        for packet in cases {
            assert_eq!(packet.to_bytes(), [packet.command_type().to_u8()]);
        }
    }

    #[test]
    fn set_device_config_matches_stm32_layout() {
        let packet = FIRMCommandPacket::build_set_device_config_command(DeviceConfig {
            name: "FIRM".to_string(),
            frequency: 50,
            protocol: DeviceProtocol::UART,
        });
        let bytes = packet.to_bytes();
        assert_eq!(bytes[0], FIRMCommand::SetDeviceConfig.to_u8());
        assert_eq!(packet.payload().len(), DEVICE_CONFIG_PAYLOAD_LENGTH);
        assert_eq!(&packet.payload()[0..2], &50u16.to_le_bytes());
        assert_eq!(&packet.payload()[2..6], b"FIRM");
        assert_eq!(&packet.payload()[34..38], &[1, 1, 0, 0]);
    }

    #[test]
    fn set_imu_calibration_payload_layout() {
        let accel_offsets = [1.0_f32, 2.0, 3.0];
        let accel_matrix = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let gyro_offsets = [-1.0_f32, -2.0, -3.0];
        let gyro_matrix = [2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0];
        let packet = FIRMCommandPacket::build_set_imu_calibration_command(
            accel_offsets,
            accel_matrix,
            gyro_offsets,
            gyro_matrix,
        );
        assert_eq!(packet.payload().len(), IMU_CALIBRATION_PAYLOAD_LENGTH);
        for (i, expected) in accel_offsets.iter().enumerate() {
            assert_eq!(f32_from_payload(packet.payload(), i), *expected);
        }
        for (i, expected) in accel_matrix.iter().enumerate() {
            assert_eq!(f32_from_payload(packet.payload(), 3 + i), *expected);
        }
        for (i, expected) in gyro_offsets.iter().enumerate() {
            assert_eq!(f32_from_payload(packet.payload(), 12 + i), *expected);
        }
        for (i, expected) in gyro_matrix.iter().enumerate() {
            assert_eq!(f32_from_payload(packet.payload(), 15 + i), *expected);
        }
    }

    #[test]
    fn mock_packet_roundtrips_as_id_plus_payload() {
        let packet = FIRMLogPacket::new(FIRMLogPacketType::HighGPacket, vec![1, 2, 3]);
        let bytes = packet.to_bytes();
        assert_eq!(bytes, [b'A', 1, 2, 3]);
        assert_eq!(FIRMLogPacket::from_bytes(&bytes).unwrap(), packet);
    }
}
