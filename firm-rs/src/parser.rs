use crate::crc::crc16_ccitt;
use serde::Serialize;
use std::collections::VecDeque;

/// Start byte sequence for packet identification. This is in little-endian format.
const START_BYTES: [u8; 2] = [0x5a, 0xa5];

/// Size of the packet header in bytes.
const HEADER_SIZE: usize = std::mem::size_of_val(&START_BYTES);

/// Size of the length field in bytes.
const LENGTH_FIELD_SIZE: usize = 2;

/// Size of the padding buffer in bytes.
const PADDING_SIZE: usize = 4;

/// Length of the payload in bytes.
const PAYLOAD_LENGTH: usize = 56;

/// Size of the CRC field in bytes.
const CRC_SIZE: usize = 2;

/// Total size of a full data packet in bytes.
const FULL_PACKET_SIZE: usize =
    HEADER_SIZE + LENGTH_FIELD_SIZE + PADDING_SIZE + PAYLOAD_LENGTH + CRC_SIZE;

/// Standard gravity in m/s².
const GRAVITY_METERS_PER_SECONDS_SQUARED: f32 = 9.80665;

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct FIRMPacket {
    pub timestamp_seconds: f64,

    pub accel_x_meters_per_s2: f32,
    pub accel_y_meters_per_s2: f32,
    pub accel_z_meters_per_s2: f32,

    pub gyro_x_radians_per_s: f32,
    pub gyro_y_radians_per_s: f32,
    pub gyro_z_radians_per_s: f32,

    pub pressure_pascals: f32,
    pub temperature_celsius: f32,

    pub mag_x_microteslas: f32,
    pub mag_y_microteslas: f32,
    pub mag_z_microteslas: f32,
}

impl FIRMPacket {
    fn from_bytes(bytes: &[u8]) -> Self {
        fn four_bytes(bytes: &[u8], idx: &mut usize) -> [u8; 4] {
            let res = [
                bytes[*idx],
                bytes[*idx + 1],
                bytes[*idx + 2],
                bytes[*idx + 3],
            ];
            *idx += 4;
            res
        }

        let mut idx = 0;

        let temperature_celsius: f32 = f32::from_le_bytes(four_bytes(bytes, &mut idx));
        let pressure_pascals: f32 = f32::from_le_bytes(four_bytes(bytes, &mut idx));

        let accel_x_meters_per_s2: f32 =
            f32::from_le_bytes(four_bytes(bytes, &mut idx)) * GRAVITY_METERS_PER_SECONDS_SQUARED;
        let accel_y_meters_per_s2: f32 =
            f32::from_le_bytes(four_bytes(bytes, &mut idx)) * GRAVITY_METERS_PER_SECONDS_SQUARED;
        let accel_z_meters_per_s2: f32 =
            f32::from_le_bytes(four_bytes(bytes, &mut idx)) * GRAVITY_METERS_PER_SECONDS_SQUARED;

        let gyro_x_radians_per_s: f32 = f32::from_le_bytes(four_bytes(bytes, &mut idx));
        let gyro_y_radians_per_s: f32 = f32::from_le_bytes(four_bytes(bytes, &mut idx));
        let gyro_z_radians_per_s: f32 = f32::from_le_bytes(four_bytes(bytes, &mut idx));

        let mag_x_microteslas: f32 = f32::from_le_bytes(four_bytes(bytes, &mut idx));
        let mag_y_microteslas: f32 = f32::from_le_bytes(four_bytes(bytes, &mut idx));
        let mag_z_microteslas: f32 = f32::from_le_bytes(four_bytes(bytes, &mut idx));

        idx += 4; // Account for padding
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

        Self {
            timestamp_seconds,
            accel_x_meters_per_s2,
            accel_y_meters_per_s2,
            accel_z_meters_per_s2,
            gyro_x_radians_per_s,
            gyro_y_radians_per_s,
            gyro_z_radians_per_s,
            pressure_pascals,
            temperature_celsius,
            mag_x_microteslas,
            mag_y_microteslas,
            mag_z_microteslas,
        }
    }
}

pub struct SerialParser {
    serial_bytes: Vec<u8>,
    serial_packets: VecDeque<FIRMPacket>,
}

impl SerialParser {
    pub fn new() -> Self {
        SerialParser {
            serial_bytes: Vec::new(),
            serial_packets: VecDeque::new(),
        }
    }

    pub fn parse_bytes(&mut self, bytes: &[u8]) {
        self.serial_bytes.extend(bytes);

        let mut pos = 0usize;
        while pos < self.serial_bytes.len().saturating_sub(1) {
            if self.serial_bytes[pos] != START_BYTES[0]
                || self.serial_bytes[pos + 1] != START_BYTES[1]
            {
                pos += 1;
                continue;
            }

            let header_start = pos;

            // Ensure we have enough space to read the packet
            if header_start + FULL_PACKET_SIZE > self.serial_bytes.len() {
                break;
            }

            let length_start = header_start + HEADER_SIZE;

            let length_bytes = &self.serial_bytes[length_start..length_start + LENGTH_FIELD_SIZE];
            let length = u16::from_le_bytes([length_bytes[0], length_bytes[1]]);

            if length as usize != PAYLOAD_LENGTH {
                pos = length_start;
                continue;
            }

            let payload_start = length_start + LENGTH_FIELD_SIZE + PADDING_SIZE;
            let crc_start = payload_start + length as usize;
            let data_to_crc = &self.serial_bytes[header_start..crc_start];
            let data_crc = crc16_ccitt(data_to_crc);
            let crc_value = u16::from_le_bytes([
                self.serial_bytes[crc_start],
                self.serial_bytes[crc_start + 1],
            ]);

            // Verify CRC
            if data_crc != crc_value {
                pos = length_start;
                continue;
            }

            let payload_slice = &self.serial_bytes[payload_start..payload_start + length as usize];

            let packet = FIRMPacket::from_bytes(payload_slice);

            self.serial_packets.push_back(packet);

            pos = crc_start + CRC_SIZE;
        }

        self.serial_bytes = self.serial_bytes[pos..].to_vec();
    }

    pub fn get_packet(&mut self) -> Option<FIRMPacket> {
        self.serial_packets.pop_front()
    }
}
