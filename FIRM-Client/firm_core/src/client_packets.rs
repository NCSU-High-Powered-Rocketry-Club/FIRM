use alloc::vec::Vec;

use crate::constants::command::{
    CALIBRATION_OFFSETS_LENGTH, CALIBRATION_SCALE_MATRIX_LENGTH, DEVICE_NAME_LENGTH, FIRMCommand,
    FREQUENCY_LENGTH, IMU_CALIBRATION_PAYLOAD_LENGTH, NUMBER_OF_CALIBRATION_OFFSETS,
    NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS,
};
use crate::constants::log_parsing::FIRMLogPacketType;
use crate::constants::packet::PacketHeader;
use crate::{
    firm_packets::*,
    framed_packet::{Framed, FramedPacket},
    utils::str_to_bytes,
};

pub struct FIRMCommandPacket {
    command_type: FIRMCommand,
    frame: FramedPacket,
}

impl FIRMCommandPacket {
    pub fn new(command_type: FIRMCommand, payload: Vec<u8>) -> Self {
        let header = PacketHeader::Command;
        let identifier = command_type as u16;
        Self {
            command_type,
            frame: FramedPacket::new(header, identifier, payload),
        }
    }

    pub fn command_type(&self) -> FIRMCommand {
        self.command_type
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
        let mut payload = Vec::with_capacity(DEVICE_NAME_LENGTH + FREQUENCY_LENGTH + 1);
        let name_bytes = str_to_bytes::<DEVICE_NAME_LENGTH>(&config.name);
        payload.extend_from_slice(&name_bytes);
        payload.extend_from_slice(&config.frequency.to_le_bytes());

        payload.push(config.protocol as u8);

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

        // Accelerometer calibration
        for offset in &accel_offsets {
            payload.extend_from_slice(&offset.to_le_bytes());
        }
        for scale in &accel_scale_matrix {
            payload.extend_from_slice(&scale.to_le_bytes());
        }

        // Gyroscope calibration
        for offset in &gyro_offsets {
            payload.extend_from_slice(&offset.to_le_bytes());
        }
        for scale in &gyro_scale_matrix {
            payload.extend_from_slice(&scale.to_le_bytes());
        }
        Self::new(FIRMCommand::SetIMUCalibration, payload)
    }

    pub fn build_get_calibration_command() -> Self {
        Self::new(FIRMCommand::GetCalibration, Vec::new())
    }
}

impl Framed for FIRMCommandPacket {
    fn frame(&self) -> &FramedPacket {
        &self.frame
    }

    /// Parses a framed command packet from raw bytes. This method is just for testing.
    fn from_bytes(bytes: &[u8]) -> Result<Self, crate::framed_packet::FrameError> {
        let frame = FramedPacket::from_bytes(bytes)?;
        let identifier = frame.identifier();
        let command_type = FIRMCommand::from_u16(identifier)?;
        Ok(Self {
            command_type,
            frame,
        })
    }
}

pub struct FIRMLogPacket {
    packet_type: FIRMLogPacketType,
    frame: FramedPacket,
}

impl FIRMLogPacket {
    pub fn new(packet_type: FIRMLogPacketType, payload: Vec<u8>) -> Self {
        let header = PacketHeader::LogSensor;
        let identifier = packet_type as u16;
        Self {
            packet_type,
            frame: FramedPacket::new(header, identifier, payload),
        }
    }

    pub fn packet_type(&self) -> FIRMLogPacketType {
        self.packet_type
    }
}

impl Framed for FIRMLogPacket {
    fn frame(&self) -> &FramedPacket {
        &self.frame
    }

    /// Parses a framed mock sensor packet from raw bytes. This method is just for testing.
    fn from_bytes(bytes: &[u8]) -> Result<Self, crate::framed_packet::FrameError> {
        let frame = FramedPacket::from_bytes(bytes)?;
        let packet_type = FIRMLogPacketType::from_u16(frame.identifier())
            .unwrap_or(FIRMLogPacketType::HeaderPacket);
        Ok(Self { packet_type, frame })
    }
}

#[cfg(test)]
mod tests {
    use super::{FIRMCommandPacket, FIRMLogPacket};
    use crate::constants::command::{
        CRC_LENGTH, DEVICE_NAME_LENGTH, FIRMCommand, FREQUENCY_LENGTH,
        IMU_CALIBRATION_PAYLOAD_LENGTH,
    };
    use crate::constants::log_parsing::FIRMLogPacketType;
    use crate::constants::packet::PacketHeader;
    use crate::firm_packets::{DeviceConfig, DeviceProtocol};
    use crate::framed_packet::Framed;
    use crate::utils::{crc16_ccitt, str_to_bytes};

    fn f32_from_payload(payload: &[u8], idx: usize) -> f32 {
        let start = idx * 4;
        let end = start + 4;
        f32::from_le_bytes(payload[start..end].try_into().unwrap())
    }

    fn crc_from_bytes(bytes: &[u8]) -> u16 {
        u16::from_le_bytes(bytes[bytes.len() - CRC_LENGTH..].try_into().unwrap())
    }

    fn calculate_crc(bytes: &[u8]) -> u16 {
        crc16_ccitt(&bytes[..bytes.len() - CRC_LENGTH])
    }

    fn header_from_bytes(bytes: &[u8]) -> u16 {
        u16::from_le_bytes(bytes[0..2].try_into().unwrap())
    }

    fn identifier_from_bytes(bytes: &[u8]) -> u16 {
        u16::from_le_bytes(bytes[2..4].try_into().unwrap())
    }

    fn assert_common_packet_invariants(bytes: &[u8]) {
        assert_eq!(crc_from_bytes(bytes), calculate_crc(bytes));
    }

    fn assert_zero_payload_command(make: fn() -> FIRMCommandPacket, expected_identifier: u16) {
        let command_packet = make().to_bytes();
        assert_common_packet_invariants(&command_packet);
        assert_eq!(
            header_from_bytes(&command_packet),
            PacketHeader::Command as u16
        );
        assert_eq!(identifier_from_bytes(&command_packet), expected_identifier);
        assert_eq!(
            u32::from_le_bytes(command_packet[4..8].try_into().unwrap()),
            0
        );
        assert_eq!(command_packet.len(), 4 + 4 + 0 + CRC_LENGTH);
    }

    #[test]
    fn test_firm_command_packet_to_bytes_zero_payload_commands() {
        let cases: &[(u16, fn() -> FIRMCommandPacket)] = &[
            (
                FIRMCommand::GetDeviceInfo as u16,
                FIRMCommandPacket::build_get_device_info_command,
            ),
            (
                FIRMCommand::GetDeviceConfig as u16,
                FIRMCommandPacket::build_get_device_config_command,
            ),
            (
                FIRMCommand::Cancel as u16,
                FIRMCommandPacket::build_cancel_command,
            ),
            (
                FIRMCommand::Reboot as u16,
                FIRMCommandPacket::build_reboot_command,
            ),
            (
                FIRMCommand::Mock as u16,
                FIRMCommandPacket::build_mock_command,
            ),
        ];

        for (identifier, make) in cases {
            assert_zero_payload_command(*make, *identifier);
        }
    }

    #[test]
    fn test_firm_command_packet_to_bytes_set_device_config() {
        let config = DeviceConfig {
            name: "FIRM".to_string(),
            frequency: 50,
            protocol: DeviceProtocol::UART,
        };

        let command_packet =
            FIRMCommandPacket::build_set_device_config_command(config.clone()).to_bytes();
        assert_common_packet_invariants(&command_packet);

        assert_eq!(
            header_from_bytes(&command_packet),
            PacketHeader::Command as u16
        );

        assert_eq!(
            identifier_from_bytes(&command_packet),
            FIRMCommand::SetDeviceConfig as u16
        );

        let payload_len = u32::from_le_bytes(command_packet[4..8].try_into().unwrap()) as usize;
        assert_eq!(payload_len, DEVICE_NAME_LENGTH + FREQUENCY_LENGTH + 1);
        assert_eq!(command_packet.len(), 4 + 4 + payload_len + CRC_LENGTH);

        let payload = &command_packet[8..8 + payload_len];
        let (got_name_bytes, rest) = payload.split_at(DEVICE_NAME_LENGTH);
        let (got_freq_bytes, got_protocol_bytes) = rest.split_at(FREQUENCY_LENGTH);

        let expected_name_bytes = str_to_bytes::<DEVICE_NAME_LENGTH>(&config.name);
        assert_eq!(got_name_bytes, &expected_name_bytes);

        let freq = u16::from_le_bytes(got_freq_bytes.try_into().unwrap());
        assert_eq!(freq, config.frequency);
        assert_eq!(got_protocol_bytes, &[0x02]);
    }

    #[test]
    fn test_firm_command_packet_to_bytes_set_imu_calibration_payload_layout() {
        let accel_offsets = [1.0_f32, 2.0_f32, 3.0_f32];
        let accel_matrix = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let gyro_offsets = [-1.0_f32, -2.0_f32, -3.0_f32];
        let gyro_matrix = [2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0];

        let command_packet = FIRMCommandPacket::build_set_imu_calibration_command(
            accel_offsets,
            accel_matrix,
            gyro_offsets,
            gyro_matrix,
        )
        .to_bytes();

        assert_common_packet_invariants(&command_packet);
        assert_eq!(
            header_from_bytes(&command_packet),
            PacketHeader::Command as u16
        );
        assert_eq!(
            identifier_from_bytes(&command_packet),
            FIRMCommand::SetIMUCalibration as u16
        );

        let payload_len = u32::from_le_bytes(command_packet[4..8].try_into().unwrap()) as usize;
        assert_eq!(payload_len, IMU_CALIBRATION_PAYLOAD_LENGTH);

        let payload = &command_packet[8..8 + payload_len];

        // Layout: [accel offsets 3][accel matrix 9][gyro offsets 3][gyro matrix 9]
        for i in 0..3 {
            assert_eq!(f32_from_payload(payload, i), accel_offsets[i]);
        }
        for i in 0..9 {
            assert_eq!(f32_from_payload(payload, 3 + i), accel_matrix[i]);
        }
        for i in 0..3 {
            assert_eq!(f32_from_payload(payload, 3 + 9 + i), gyro_offsets[i]);
        }
        for i in 0..9 {
            assert_eq!(f32_from_payload(payload, 3 + 9 + 3 + i), gyro_matrix[i]);
        }
    }

    #[test]
    fn test_firm_mock_packet_new() {
        let payload = vec![1u8, 2, 3];
        let packet = FIRMLogPacket::new(FIRMLogPacketType::BarometerPacket, payload.clone());
        assert_eq!(packet.header(), PacketHeader::LogSensor);
        assert_eq!(packet.packet_type(), FIRMLogPacketType::BarometerPacket);
        assert_eq!(packet.len(), payload.len() as u32);
        assert_eq!(packet.payload(), payload.as_slice());
    }

    #[test]
    fn test_firm_mock_packet_to_bytes() {
        let payload: Vec<u8> = vec![0x10, 0x20, 0x30, 0x40, 0x50];
        let packet = FIRMLogPacket::new(FIRMLogPacketType::IMUPacket, payload);
        let bytes = packet.to_bytes();
        assert_eq!(header_from_bytes(&bytes), PacketHeader::LogSensor.as_u16());
        assert_eq!(identifier_from_bytes(&bytes), b'I' as u16);
        assert_eq!(
            u32::from_le_bytes(bytes[4..8].try_into().unwrap()),
            packet.len()
        );
        assert_eq!(
            u16::from_le_bytes(bytes[bytes.len() - 2..].try_into().unwrap()),
            packet.crc()
        );
        assert_eq!(&bytes[8..bytes.len() - 2], packet.payload());
        assert_eq!(crc_from_bytes(&bytes), calculate_crc(&bytes));
    }

    #[test]
    fn test_firm_mock_packet_roundtrip_from_bytes() {
        let payload = vec![9u8, 8, 7];
        let packet = FIRMLogPacket::new(FIRMLogPacketType::HeaderPacket, payload);
        let bytes = packet.to_bytes();
        let parsed = FIRMLogPacket::from_bytes(&bytes).unwrap();
        assert_eq!(parsed.header(), PacketHeader::LogSensor);
        assert_eq!(parsed.packet_type(), FIRMLogPacketType::HeaderPacket);
        assert_eq!(parsed.len() as usize, parsed.payload().len());
        assert_eq!(parsed.payload(), packet.payload());
        assert_eq!(parsed.crc(), packet.crc());
    }
}
