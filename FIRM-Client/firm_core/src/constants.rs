pub mod command {
    use crate::wire_packet::PacketError;

    #[repr(u8)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum FIRMCommand {
        GetDeviceInfo = 0x02,
        GetDeviceConfig = 0x03,
        SetDeviceConfig = 0x04,
        Reboot = 0x05,
        Mock = 0x06,
        SetMagnetometerCalibration = 0x07,
        SetIMUCalibration = 0x08,
        GetCalibration = 0x09,
        Cancel = 0x0A,
    }

    impl FIRMCommand {
        pub const fn to_u8(self) -> u8 {
            self as u8
        }

        pub const fn from_u8(identifier: u8) -> Result<Self, PacketError> {
            match identifier {
                id if id == FIRMCommand::GetDeviceInfo.to_u8() => Ok(FIRMCommand::GetDeviceInfo),
                id if id == FIRMCommand::GetDeviceConfig.to_u8() => {
                    Ok(FIRMCommand::GetDeviceConfig)
                }
                id if id == FIRMCommand::SetDeviceConfig.to_u8() => {
                    Ok(FIRMCommand::SetDeviceConfig)
                }
                id if id == FIRMCommand::Reboot.to_u8() => Ok(FIRMCommand::Reboot),
                id if id == FIRMCommand::Mock.to_u8() => Ok(FIRMCommand::Mock),
                id if id == FIRMCommand::SetMagnetometerCalibration.to_u8() => {
                    Ok(FIRMCommand::SetMagnetometerCalibration)
                }
                id if id == FIRMCommand::SetIMUCalibration.to_u8() => {
                    Ok(FIRMCommand::SetIMUCalibration)
                }
                id if id == FIRMCommand::GetCalibration.to_u8() => Ok(FIRMCommand::GetCalibration),
                id if id == FIRMCommand::Cancel.to_u8() => Ok(FIRMCommand::Cancel),
                _ => Err(PacketError::UnknownIdentifier(identifier)),
            }
        }

        pub const fn command_payload_len(self) -> usize {
            match self {
                Self::GetDeviceInfo
                | Self::GetDeviceConfig
                | Self::Reboot
                | Self::Mock
                | Self::GetCalibration
                | Self::Cancel => 0,
                Self::SetDeviceConfig => DEVICE_CONFIG_PAYLOAD_LENGTH,
                Self::SetMagnetometerCalibration => CALIBRATION_PAYLOAD_LENGTH,
                Self::SetIMUCalibration => IMU_CALIBRATION_PAYLOAD_LENGTH,
            }
        }

        pub const fn response_payload_len(self) -> Option<usize> {
            match self {
                Self::GetDeviceInfo => Some(DEVICE_INFO_PAYLOAD_LENGTH),
                Self::GetDeviceConfig => Some(DEVICE_CONFIG_PAYLOAD_LENGTH),
                Self::SetDeviceConfig
                | Self::Mock
                | Self::SetMagnetometerCalibration
                | Self::SetIMUCalibration
                | Self::Cancel => Some(ACKNOWLEDGEMENT_PAYLOAD_LENGTH),
                Self::GetCalibration => Some(ALL_CALIBRATIONS_PAYLOAD_LENGTH),
                Self::Reboot => None,
            }
        }
    }

    pub const IDENTIFIER_LENGTH: usize = 1;
    pub const DEVICE_NAME_LENGTH: usize = 32;
    pub const DEVICE_ID_LENGTH: usize = 8;
    pub const FIRMWARE_VERSION_LENGTH: usize = 8;
    pub const FREQUENCY_LENGTH: usize = 2;
    pub const MIN_DEVICE_FREQUENCY_HZ: u16 = 1;
    pub const MAX_DEVICE_FREQUENCY_HZ: u16 = 1000;
    pub const COMMUNICATION_FLAGS_LENGTH: usize = 4;
    pub const ACKNOWLEDGEMENT_PAYLOAD_LENGTH: usize = 1;
    pub const DEVICE_INFO_PAYLOAD_LENGTH: usize = DEVICE_ID_LENGTH + FIRMWARE_VERSION_LENGTH;
    pub const DEVICE_CONFIG_PAYLOAD_LENGTH: usize =
        FREQUENCY_LENGTH + DEVICE_NAME_LENGTH + COMMUNICATION_FLAGS_LENGTH;
    pub const DATA_PACKET_ID: u8 = 0x01;
    pub const FIRM_DATA_PAYLOAD_LENGTH: usize = 8 + 20 * 4;
    pub const FIRM_DATA_MESSAGE_LENGTH: usize = IDENTIFIER_LENGTH + FIRM_DATA_PAYLOAD_LENGTH;
    pub const NUMBER_OF_CALIBRATION_OFFSETS: usize = 3;
    pub const NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS: usize = 9;
    pub const CALIBRATION_OFFSETS_LENGTH: usize = NUMBER_OF_CALIBRATION_OFFSETS * 4;
    pub const CALIBRATION_SCALE_MATRIX_LENGTH: usize =
        NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS * 4;
    pub const CALIBRATION_PAYLOAD_LENGTH: usize =
        CALIBRATION_OFFSETS_LENGTH + CALIBRATION_SCALE_MATRIX_LENGTH;
    pub const ALL_CALIBRATIONS_PAYLOAD_LENGTH: usize = CALIBRATION_PAYLOAD_LENGTH * 4;

    /// IMU calibration includes both accelerometer and gyroscope calibration.
    ///
    /// Payload layout: [accel offsets (3 f32)][accel matrix (9 f32)][gyro offsets (3 f32)][gyro matrix (9 f32)]
    pub const NUMBER_OF_IMU_CALIBRATION_SETS: usize = 2;
    pub const IMU_CALIBRATION_PAYLOAD_LENGTH: usize = (CALIBRATION_OFFSETS_LENGTH
        + CALIBRATION_SCALE_MATRIX_LENGTH)
        * NUMBER_OF_IMU_CALIBRATION_SETS;
}

pub mod log_parsing {
    use std::time::Duration;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct LogPacketMeta {
        pub id: u8,
        pub packet_type: FIRMLogPacketType,
        pub payload_size: usize,
        pub name: &'static str,
    }

    pub const LOG_PACKET_META: [LogPacketMeta; 4] = [
        LogPacketMeta {
            id: BMP581_ID,
            packet_type: FIRMLogPacketType::BarometerPacket,
            payload_size: BMP581_SIZE,
            name: "BMP581",
        },
        LogPacketMeta {
            id: ICM45686_ID,
            packet_type: FIRMLogPacketType::IMUPacket,
            payload_size: ICM45686_SIZE,
            name: "ICM45686",
        },
        LogPacketMeta {
            id: MMC5983MA_ID,
            packet_type: FIRMLogPacketType::MagnetometerPacket,
            payload_size: MMC5983MA_SIZE,
            name: "MMC5983MA",
        },
        LogPacketMeta {
            id: ADXL371_ID,
            packet_type: FIRMLogPacketType::HighGPacket,
            payload_size: ADXL371_SIZE,
            name: "ADXL371",
        },
    ];

    pub fn log_packet_meta_from_id(id: u8) -> Option<&'static LogPacketMeta> {
        LOG_PACKET_META.iter().find(|m| m.id == id)
    }

    /// One-byte identifier placed at the start of each mock sensor message.
    #[repr(u8)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum FIRMLogPacketType {
        HeaderPacket = HEADER_ID,
        BarometerPacket = BMP581_ID,
        IMUPacket = ICM45686_ID,
        MagnetometerPacket = MMC5983MA_ID,
        HighGPacket = ADXL371_ID,
    }

    impl FIRMLogPacketType {
        pub const fn as_u8(self) -> u8 {
            self as u8
        }

        pub const fn from_u8(v: u8) -> Option<Self> {
            match v {
                v if v == Self::HeaderPacket as u8 => Some(Self::HeaderPacket),
                v if v == Self::BarometerPacket as u8 => Some(Self::BarometerPacket),
                v if v == Self::IMUPacket as u8 => Some(Self::IMUPacket),
                v if v == Self::MagnetometerPacket as u8 => Some(Self::MagnetometerPacket),
                v if v == Self::HighGPacket as u8 => Some(Self::HighGPacket),
                _ => None,
            }
        }

        pub const fn as_char(self) -> char {
            // SAFETY: All enum variants are valid ASCII values.
            self as u8 as char
        }
    }

    pub const HEADER_ID: u8 = b'H';
    pub const BMP581_ID: u8 = b'B';
    pub const ICM45686_ID: u8 = b'I';
    pub const MMC5983MA_ID: u8 = b'M';
    pub const ADXL371_ID: u8 = b'A';

    // The length of the payloads not including the 4 byte timestamp
    pub const BMP581_SIZE: usize = 6;
    pub const ICM45686_SIZE: usize = 15;
    pub const MMC5983MA_SIZE: usize = 7;
    pub const ADXL371_SIZE: usize = 6;

    pub const LOG_FILE_EOF_PADDING_LENGTH: usize = 20;
    pub const LOG_PACKET_TIMESTAMP_SIZE: usize = 4;

    pub const HEADER_SIZE_TEXT: usize = 14; // "FIRM LOG vx.x"
    pub const HEADER_UID_SIZE: usize = 8;
    pub const HEADER_DEVICE_NAME_LEN: usize = 32;
    pub const HEADER_COMM_SIZE: usize = 4; // 1 byte usb, 1 byte uart, 1 byte spi, 1 byte i2c
    pub const HEADER_FIRMWARE_VERSION_SIZE: usize = 8; // "vX.X.X.X"
    pub const HEADER_FREQUENCY_SIZE: usize = 2;
    pub const HEADER_CAL_SIZE: usize = 4 * (3 + 9) * 4; // 4 calibration blocks, each 3 offsets + 3x3 matrix
    pub const HEADER_TOTAL_SIZE: usize = HEADER_SIZE_TEXT
        + HEADER_UID_SIZE
        + HEADER_DEVICE_NAME_LEN
        + HEADER_COMM_SIZE
        + HEADER_FIRMWARE_VERSION_SIZE
        + HEADER_FREQUENCY_SIZE
        + HEADER_CAL_SIZE;

    pub const HEADER_PARSE_DELAY: Duration = Duration::from_millis(100);
}

pub mod data_deriver_constants {
    pub struct DataDeriverConstants;

    impl DataDeriverConstants {
        pub const SPECIFIC_HEAT_RATIO_AIR: f32 = 1.4;
        pub const SPECIFIC_GAS_CONSTANT_AIR_J_PER_KG_K: f32 = 287.05;
        pub const CELSIUS_TO_KELVIN_OFFSET: f32 = 273.15;
        pub const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;
        pub const IMU_Z_AXIS_CCW_ROTATION_DEGREES: f32 = 45.0;
    }
}
