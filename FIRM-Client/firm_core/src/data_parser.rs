use alloc::collections::VecDeque;
use alloc::vec::Vec;

use crate::constants::command::{DATA_PACKET_ID, FIRM_DATA_MESSAGE_LENGTH, FIRMCommand};
use crate::data_processor::DataProcessor;
use crate::firm_packets::{FIRMDataPacket, FIRMResponsePacket, ProcessedFIRMData};

/// Streaming parser for the STM32 USB format `[id: u8][fixed-size payload]`.
pub struct SerialParser {
    serial_bytes: Vec<u8>,
    parsed_data_packets: VecDeque<ProcessedFIRMData>,
    parsed_response_packets: VecDeque<FIRMResponsePacket>,
    expected_responses: Vec<FIRMCommand>,
    data_synchronized: bool,
    data_processor: DataProcessor,
}

impl SerialParser {
    pub fn new() -> Self {
        Self {
            serial_bytes: Vec::new(),
            parsed_data_packets: VecDeque::new(),
            parsed_response_packets: VecDeque::new(),
            expected_responses: Vec::new(),
            data_synchronized: false,
            data_processor: DataProcessor::default(),
        }
    }

    /// Registers a command response that may be accepted from the raw byte stream.
    pub fn expect_response(&mut self, command: FIRMCommand) -> bool {
        if command.response_payload_len().is_none() {
            return false;
        }
        self.expected_responses.push(command);
        true
    }

    pub fn parse_bytes(&mut self, bytes: &[u8]) {
        self.serial_bytes.extend_from_slice(bytes);

        loop {
            let Some(&identifier) = self.serial_bytes.first() else {
                break;
            };

            let message_len = if identifier == DATA_PACKET_ID {
                if !self.data_synchronized {
                    if let Some(offset) = self.find_valid_expected_response_offset() {
                        self.serial_bytes.drain(..offset);
                        continue;
                    }
                    if let Some(offset) = self.find_data_sync_offset() {
                        self.serial_bytes.drain(..offset);
                        self.data_synchronized = true;
                        continue;
                    }

                    // Retain enough bytes to recognize two consecutive packet starts.
                    let retain = FIRM_DATA_MESSAGE_LENGTH * 2 - 1;
                    if self.serial_bytes.len() > retain {
                        let discard = self.serial_bytes.len() - retain;
                        self.serial_bytes.drain(..discard);
                    }
                    break;
                }
                FIRM_DATA_MESSAGE_LENGTH
            } else {
                let Ok(command) = FIRMCommand::from_u8(identifier) else {
                    // With no CRC or length field, the only possible resynchronization is
                    // to discard bytes until a known message identifier is found.
                    self.serial_bytes.remove(0);
                    continue;
                };
                if !self.expected_responses.contains(&command) {
                    self.data_synchronized = false;
                    self.serial_bytes.remove(0);
                    continue;
                }
                let Some(payload_len) = command.response_payload_len() else {
                    self.serial_bytes.remove(0);
                    continue;
                };
                1 + payload_len
            };

            if self.serial_bytes.len() < message_len {
                if let Some(offset) = self.find_valid_expected_response_offset() {
                    self.serial_bytes.drain(..offset);
                    continue;
                }
                break;
            }

            if identifier == DATA_PACKET_ID {
                let packet = FIRMDataPacket::from_bytes(&self.serial_bytes[..message_len]);
                if let Ok(packet) = packet {
                    self.serial_bytes.drain(..message_len);
                    self.parsed_data_packets
                        .push_back(self.data_processor.process_firm_data(packet.data()));
                } else {
                    self.data_synchronized = false;
                    self.serial_bytes.remove(0);
                }
            } else {
                let packet = FIRMResponsePacket::from_bytes(&self.serial_bytes[..message_len]);
                if let Ok(packet) = packet {
                    self.serial_bytes.drain(..message_len);
                    // A validated response has a known fixed length, so its end is
                    // an exact wire-message boundary. The next byte can be consumed
                    // as a packet identifier without waiting for heuristic recovery.
                    self.data_synchronized = true;
                    if let Some(index) = self
                        .expected_responses
                        .iter()
                        .position(|command| command.to_u8() == identifier)
                    {
                        self.expected_responses.remove(index);
                    }
                    self.parsed_response_packets.push_back(packet);
                } else {
                    self.serial_bytes.remove(0);
                }
            }
        }
    }

    pub fn get_data_packet(&mut self) -> Option<ProcessedFIRMData> {
        self.parsed_data_packets.pop_front()
    }

    pub fn get_response_packet(&mut self) -> Option<FIRMResponsePacket> {
        self.parsed_response_packets.pop_front()
    }

    fn find_valid_expected_response_offset(&self) -> Option<usize> {
        self.serial_bytes
            .iter()
            .enumerate()
            .skip(1)
            .find_map(|(offset, identifier)| {
                let command = FIRMCommand::from_u8(*identifier).ok()?;
                if !self.expected_responses.contains(&command) {
                    return None;
                }
                let message_len = 1 + command.response_payload_len()?;
                let end = offset + message_len;
                if end > self.serial_bytes.len() {
                    return None;
                }
                FIRMResponsePacket::from_bytes(&self.serial_bytes[offset..end])
                    .ok()
                    .map(|_| offset)
            })
    }

    fn find_data_sync_offset(&self) -> Option<usize> {
        if self.serial_bytes.len() <= FIRM_DATA_MESSAGE_LENGTH {
            return None;
        }

        (0..self.serial_bytes.len() - FIRM_DATA_MESSAGE_LENGTH).find(|offset| {
            self.serial_bytes[*offset] == DATA_PACKET_ID
                && self.serial_bytes[*offset + FIRM_DATA_MESSAGE_LENGTH] == DATA_PACKET_ID
                && self.timestamps_form_data_sequence(*offset)
        })
    }

    fn timestamps_form_data_sequence(&self, offset: usize) -> bool {
        let first_start = offset + 1;
        let second_start = offset + FIRM_DATA_MESSAGE_LENGTH + 1;
        if second_start + 8 > self.serial_bytes.len() {
            return false;
        }
        let first = f64::from_le_bytes(
            self.serial_bytes[first_start..first_start + 8]
                .try_into()
                .unwrap(),
        );
        let second = f64::from_le_bytes(
            self.serial_bytes[second_start..second_start + 8]
                .try_into()
                .unwrap(),
        );
        let timestamp_is_plausible =
            |value: f64| value.is_finite() && value >= 0.0 && (value == 0.0 || value >= 1.0e-6);
        timestamp_is_plausible(first)
            && timestamp_is_plausible(second)
            && second >= first
            && second - first <= 60.0
            && (second > first || (first == 0.0 && second == 0.0))
    }
}

impl Default for SerialParser {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::SerialParser;
    use crate::constants::command::{DATA_PACKET_ID, FIRM_DATA_PAYLOAD_LENGTH, FIRMCommand};

    fn telemetry_message(timestamp: f64, temperature: f32, pressure: f32) -> Vec<u8> {
        let mut payload = vec![0u8; FIRM_DATA_PAYLOAD_LENGTH];
        payload[0..8].copy_from_slice(&timestamp.to_le_bytes());
        payload[8..12].copy_from_slice(&temperature.to_le_bytes());
        payload[12..16].copy_from_slice(&pressure.to_le_bytes());
        payload[72..76].copy_from_slice(&1.0f32.to_le_bytes());

        let mut message = vec![DATA_PACKET_ID];
        message.extend_from_slice(&payload);
        message
    }

    #[test]
    fn parses_raw_data_packet_split_across_calls() {
        let mut payload = vec![0u8; FIRM_DATA_PAYLOAD_LENGTH];
        payload[0..8].copy_from_slice(&42.0f64.to_le_bytes());
        payload[8..12].copy_from_slice(&25.0f32.to_le_bytes());
        payload[72..76].copy_from_slice(&1.0f32.to_le_bytes());
        let mut message = vec![DATA_PACKET_ID];
        message.extend_from_slice(&payload);

        let mut parser = SerialParser::new();
        parser.parse_bytes(&message[..20]);
        assert!(parser.get_data_packet().is_none());
        parser.parse_bytes(&message[20..]);
        assert!(parser.get_data_packet().is_none());
        parser.parse_bytes(&telemetry_message(43.0, 26.0, 100_000.0));

        let packet = parser.get_data_packet().expect("expected one data packet");
        assert_eq!(packet.timestamp_seconds, 42.0);
        assert_eq!(packet.temperature_celsius, 25.0);
    }

    #[test]
    fn parses_response_and_following_telemetry_in_one_chunk() {
        let mut bytes = vec![FIRMCommand::SetDeviceConfig.to_u8(), 1, DATA_PACKET_ID];
        bytes.extend_from_slice(&[0u8; FIRM_DATA_PAYLOAD_LENGTH]);

        let mut parser = SerialParser::new();
        parser.expect_response(FIRMCommand::SetDeviceConfig);
        parser.parse_bytes(&bytes);
        assert!(parser.get_response_packet().is_some());
        assert!(parser.get_data_packet().is_some());
    }

    #[test]
    fn discards_unknown_bytes_before_a_message() {
        let bytes = vec![0xFF, 0x00, FIRMCommand::Cancel.to_u8(), 1];
        let mut parser = SerialParser::new();
        parser.expect_response(FIRMCommand::Cancel);
        parser.parse_bytes(&bytes);
        assert!(parser.get_response_packet().is_some());
    }

    #[test]
    fn resynchronizes_when_stream_starts_on_payload_byte_equal_to_data_id() {
        // The low byte of this valid f64 timestamp is 0x01. If the serial stream
        // starts one byte after the real packet ID, a parser that trusts the first
        // 0x01 will decode every field at a one-byte offset.
        let first = telemetry_message(f64::from_bits(42.0f64.to_bits() + 1), 25.0, 101_325.0);
        assert_eq!(first[1], DATA_PACKET_ID);
        let second = telemetry_message(43.0, 26.0, 100_000.0);
        let third = telemetry_message(44.0, 27.0, 99_500.0);

        let mut parser = SerialParser::new();
        parser.parse_bytes(&first[1..]);
        parser.parse_bytes(&second);
        parser.parse_bytes(&third);

        let packet = parser
            .get_data_packet()
            .expect("expected parser to recover at the next complete telemetry message");
        assert_eq!(packet.timestamp_seconds, 43.0);
        assert_eq!(packet.temperature_celsius, 26.0);
        assert_eq!(packet.pressure_pascals, 100_000.0);
        assert_eq!(
            parser
                .get_data_packet()
                .expect("expected following telemetry packet")
                .timestamp_seconds,
            44.0
        );
        assert!(parser.get_data_packet().is_none());
    }

    #[test]
    fn accepts_sensor_data_when_estimator_quaternion_is_transient() {
        let mut message = telemetry_message(44.0, 27.0, 99_500.0);
        message[1 + 72..1 + 76].copy_from_slice(&0.1f32.to_le_bytes());

        let mut parser = SerialParser::new();
        parser.parse_bytes(&message);
        parser.parse_bytes(&telemetry_message(45.0, 28.0, 99_000.0));

        let packet = parser
            .get_data_packet()
            .expect("raw sensor telemetry must not depend on estimator normalization");
        assert_eq!(packet.timestamp_seconds, 44.0);
        assert_eq!(packet.temperature_celsius, 27.0);
        assert_eq!(packet.pressure_pascals, 99_500.0);
    }

    #[test]
    fn rejects_config_response_identifier_found_inside_live_telemetry() {
        // This is the failure observed in the web app: attachment begins on a 0x03
        // telemetry payload byte and the following bytes begin 01 08. Treating that
        // byte as GetDeviceConfig decodes frequency 0x0801 == 2049 Hz and arbitrary
        // payload bytes as the name and transport flags.
        let mut false_config = vec![FIRMCommand::GetDeviceConfig.to_u8(), 0x01, 0x08];
        false_config.resize(1 + 38, 0x41);
        false_config[35] = 0;
        false_config[36] = 0;
        false_config[37] = 0;
        false_config[38] = 1; // Makes the bogus response appear to select SPI.

        let mut real_config = vec![FIRMCommand::GetDeviceConfig.to_u8()];
        real_config.extend_from_slice(&100u16.to_le_bytes());
        real_config.extend_from_slice(b"FIRM");
        real_config.resize(1 + 2 + 32, 0);
        real_config.extend_from_slice(&[1, 0, 0, 0]);

        let mut parser = SerialParser::new();
        parser.expect_response(FIRMCommand::GetDeviceConfig);
        parser.parse_bytes(&false_config);
        parser.parse_bytes(&real_config);

        let response = parser
            .get_response_packet()
            .expect("expected the real configuration response");
        let crate::firm_packets::FIRMResponse::GetDeviceConfig(config) = response.response() else {
            panic!("expected GetDeviceConfig response");
        };
        assert_eq!(config.name, "FIRM");
        assert_eq!(config.frequency, 100);
        assert_eq!(config.protocol, crate::firm_packets::DeviceProtocol::USB);
        assert!(parser.get_response_packet().is_none());
    }
}
