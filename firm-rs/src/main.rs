use std::{collections::VecDeque, process::exit, time::Duration};

/// Start byte sequence for packet identification. This is in little-endian format.
const START_BYTES: [u8; 2] = [0x5a, 0xa5];

/// Size of the packet header in bytes.
const HEADER_SIZE: usize = size_of_val(&START_BYTES);

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

const CRC16_TABLE: [u16; 256] = [
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF, 0x8C48, 0x9DC1, 0xAF5A, 0xBED3,
    0xCA6C, 0xDBE5, 0xE97E, 0xF8F7, 0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876, 0x2102, 0x308B, 0x0210, 0x1399,
    0x6726, 0x76AF, 0x4434, 0x55BD, 0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C, 0xBDCB, 0xAC42, 0x9ED9, 0x8F50,
    0xFBEF, 0xEA66, 0xD8FD, 0xC974, 0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3, 0x5285, 0x430C, 0x7197, 0x601E,
    0x14A1, 0x0528, 0x37B3, 0x263A, 0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9, 0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5,
    0xA96A, 0xB8E3, 0x8A78, 0x9BF1, 0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70, 0x8408, 0x9581, 0xA71A, 0xB693,
    0xC22C, 0xD3A5, 0xE13E, 0xF0B7, 0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036, 0x18C1, 0x0948, 0x3BD3, 0x2A5A,
    0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E, 0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD, 0xB58B, 0xA402, 0x9699, 0x8710,
    0xF3AF, 0xE226, 0xD0BD, 0xC134, 0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3, 0x4A44, 0x5BCD, 0x6956, 0x78DF,
    0x0C60, 0x1DE9, 0x2F72, 0x3EFB, 0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A, 0xE70E, 0xF687, 0xC41C, 0xD595,
    0xA12A, 0xB0A3, 0x8238, 0x93B1, 0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330, 0x7BC7, 0x6A4E, 0x58D5, 0x495C,
    0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
];

fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0x0000;
    for &byte in data {
        let idx: u8 = crc as u8 ^ byte;
        crc = CRC16_TABLE[idx as usize] ^ (crc >> 8);
    }
    return crc;
}

#[derive(Debug, Clone, PartialEq)]
struct FIRMPacket {
    timestamp_seconds: f64,

    accel_x_meters_per_s2: f32,
    accel_y_meters_per_s2: f32,
    accel_z_meters_per_s2: f32,

    gyro_x_radians_per_s: f32,
    gyro_y_radians_per_s: f32,
    gyro_z_radians_per_s: f32,

    pressure_pascals: f32,
    temperature_celsius: f32,

    mag_x_microteslas: f32,
    mag_y_microteslas: f32,
    mag_z_microteslas: f32,
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

        let accel_x_meters_per_s2: f32 = f32::from_le_bytes(four_bytes(bytes, &mut idx));
        let accel_y_meters_per_s2: f32 = f32::from_le_bytes(four_bytes(bytes, &mut idx));
        let accel_z_meters_per_s2: f32 = f32::from_le_bytes(four_bytes(bytes, &mut idx));

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

struct SerialParser {
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

        let mut pos = 0;
        while pos < self.serial_bytes.len() - 1 {
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

fn main() {
    let ports = serialport::available_ports().expect("No ports found!");

    if ports.is_empty() {
        eprintln!("No serial ports detected");
        exit(1);
    }

    if ports.len() > 1 {
        eprintln!("Too many serial ports detected");
        exit(1);
    }

    let port_info: &serialport::SerialPortInfo = &ports[0];

    let mut port = serialport::new(port_info.port_name.clone(), 115_200)
        .timeout(Duration::from_millis(10))
        .open()
        .expect("Failed to open port");

    let mut parser = SerialParser::new();

    loop {
        let mut buf = [0; 1024];
        let num_bytes = port.read(&mut buf).unwrap();
        let slice = &buf[0..num_bytes];
        parser.parse_bytes(slice);

        while let Some(p) = parser.get_packet() {
            println!("{p:#?}");
        }
    }
}
