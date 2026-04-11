use alloc::collections::VecDeque;
use alloc::vec::Vec;

use crate::client_packets::FIRMLogPacket;
use crate::constants::log_parsing::FIRMLogPacketType;
use crate::constants::log_parsing::*;

pub struct LogParser {
    /// Rolling buffer of unprocessed bytes.
    bytes: Vec<u8>,
    /// Queue of parsed log packets and their inter-packet delay.
    parsed_packets: VecDeque<(FIRMLogPacket, f64)>,

    // Log header state.
    header_parsed: bool,

    // Timestamp state (clock-count based).
    last_clock_count: Option<u32>,

    // Whitespace repeat counter (used by the Python decoder to detect end-of-data).
    num_repeat_whitespace: usize,

    // End-of-data detected (via whitespace padding).
    eof_reached: bool,
}

impl Default for LogParser {
    fn default() -> Self {
        Self::new()
    }
}

impl LogParser {
    /// Creates a new empty `LogParser`.
    pub fn new() -> Self {
        Self {
            bytes: Vec::new(),
            parsed_packets: VecDeque::new(),
            header_parsed: false,
            last_clock_count: None,
            num_repeat_whitespace: 0,
            eof_reached: false,
        }
    }

    /// Reads the log header and initializes scale factors.
    pub fn read_header(&mut self, header_bytes: &[u8]) {
        assert_eq!(header_bytes.len(), HEADER_TOTAL_SIZE);

        // Reset streaming state for a fresh playback run.
        self.bytes.clear();
        self.parsed_packets.clear();
        self.last_clock_count = None;
        self.num_repeat_whitespace = 0;
        self.eof_reached = false;

        self.header_parsed = true;
    }

    /// Feeds a new chunk of bytes into the parser.
    ///
    /// Parses as many log packets as possible and enqueues framed log packets.
    ///
    /// This code is just copied from the Python decoder script.
    pub fn parse_bytes(&mut self, chunk: &[u8]) {
        if self.eof_reached {
            return;
        }

        self.bytes.extend_from_slice(chunk);

        // Parse log packets
        let mut position = 0usize;
        while position < self.bytes.len() {
            let log_packet_start = position;

            let id = self.bytes[position];
            if id == 0 {
                // whitespace padding between log packets
                self.num_repeat_whitespace += 1;
                // End-of-data if whitespace repeats enough times, matching the Python decoder.
                if self.num_repeat_whitespace > LOG_FILE_EOF_PADDING_LENGTH {
                    // Treat as EOF padding; drop buffered bytes.
                    self.bytes.clear();
                    self.eof_reached = true;
                    break;
                }
                position += 1;
                continue;
            }
            self.num_repeat_whitespace = 0;

            // Need timestamp.
            if position + 1 + LOG_PACKET_TIMESTAMP_SIZE > self.bytes.len() {
                position = log_packet_start;
                break;
            }

            position += 1;
            let timestamp = &self.bytes[position..position + LOG_PACKET_TIMESTAMP_SIZE];
            position += LOG_PACKET_TIMESTAMP_SIZE;
            let clock_count = u32::from_le_bytes(
                timestamp
                    .try_into()
                    .expect("timestamp slice length mismatch"),
            );

            // Compute the delay from the previous log packet timestamp.
            // The log clock is 32-bit and ticks at 168 MHz.
            let delay_seconds = match self.last_clock_count {
                None => 0.0,
                Some(prev) => {
                    let delta = clock_count.wrapping_sub(prev);
                    (delta as f64) / 168e6
                }
            };

            self.last_clock_count = Some(clock_count);

            let (packet_type, size) = match id {
                BMP581_ID => (FIRMLogPacketType::BarometerPacket, BMP581_SIZE),
                ICM45686_ID => (FIRMLogPacketType::IMUPacket, ICM45686_SIZE),
                MMC5983MA_ID => (FIRMLogPacketType::MagnetometerPacket, MMC5983MA_SIZE),
                _ => {
                    // Unknown/garbage byte. Don't give up immediately: advance by one byte and
                    // keep scanning so we can re-sync if we're offset or the file has junk.
                    position = log_packet_start + 1;
                    continue;
                }
            };

            if position + size > self.bytes.len() {
                position = log_packet_start;
                break;
            }

            let raw = &self.bytes[position..position + size];
            position += size;

            let mut payload = Vec::with_capacity(LOG_PACKET_TIMESTAMP_SIZE + size);
            payload.extend_from_slice(timestamp);
            payload.extend_from_slice(raw);
            let pkt = FIRMLogPacket::new(packet_type, payload);
            self.parsed_packets.push_back((pkt, delay_seconds));
        }

        if position >= self.bytes.len() {
            self.bytes.clear();
            return;
        }

        self.bytes = self.bytes[position..].to_vec();
    }

    /// Pops the next parsed log packet and returns it with its delay since the last one.
    pub fn get_packet_and_time_delay(&mut self) -> Option<(FIRMLogPacket, f64)> {
        self.parsed_packets.pop_front()
    }

    /// Returns true once end-of-data padding is detected.
    pub fn eof_reached(&self) -> bool {
        self.eof_reached
    }

    /// Pops the next parsed log packet (no delay info).
    pub fn get_packet(&mut self) -> Option<FIRMLogPacket> {
        self.parsed_packets.pop_front().map(|(pkt, _)| pkt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::framed_packet::Framed;

    fn make_header() -> Vec<u8> {
        let mut header = Vec::new();
        header.resize(HEADER_TOTAL_SIZE, 0u8);
        header
    }

    fn make_log_packet_bytes(id: u8, clock_count: u32, raw_len: usize) -> Vec<u8> {
        // Timestamp is stored as a 32-bit little-endian counter (4 bytes).
        let le = clock_count.to_le_bytes();

        let mut out = vec![0u8; 1 + LOG_PACKET_TIMESTAMP_SIZE + raw_len];
        out[0] = id;
        out[1..(1 + LOG_PACKET_TIMESTAMP_SIZE)].copy_from_slice(&le);
        out
    }

    #[test]
    fn test_reads_header_and_packet() {
        let header = make_header();
        let log_packet_bytes = make_log_packet_bytes(ICM45686_ID, 1, ICM45686_SIZE);

        let mut parser = LogParser::new();
        parser.read_header(&header);
        parser.parse_bytes(&log_packet_bytes);

        let (log_packet, delay) = parser.get_packet_and_time_delay().unwrap();
        assert_eq!(delay, 0.0);
        assert_eq!(log_packet.packet_type(), FIRMLogPacketType::IMUPacket);
        assert_eq!(
            log_packet.payload().len(),
            LOG_PACKET_TIMESTAMP_SIZE + ICM45686_SIZE
        );
        assert_eq!(log_packet.len() as usize, log_packet.payload().len());
        assert_eq!(
            &log_packet.payload()[0..LOG_PACKET_TIMESTAMP_SIZE],
            &[0x01, 0x00, 0x00, 0x00]
        );
        assert!(parser.get_packet_and_time_delay().is_none());
    }

    #[test]
    fn test_delay_works() {
        let header = make_header();

        // delta = 168 ticks => delay = 1e-6 seconds
        let log_packet_bytes1 = make_log_packet_bytes(BMP581_ID, 1000, BMP581_SIZE);
        let log_packet_bytes2 = make_log_packet_bytes(BMP581_ID, 1168, BMP581_SIZE);

        let mut bytes = Vec::new();
        bytes.extend_from_slice(&log_packet_bytes1);
        bytes.extend_from_slice(&log_packet_bytes2);

        let mut parser = LogParser::new();
        parser.read_header(&header);
        parser.parse_bytes(&bytes);

        let (log_packet1, d1) = parser.get_packet_and_time_delay().unwrap();
        let (log_packet2, d2) = parser.get_packet_and_time_delay().unwrap();
        assert_eq!(d1, 0.0);

        assert_eq!(
            log_packet1.packet_type(),
            FIRMLogPacketType::BarometerPacket
        );
        assert_eq!(
            log_packet2.packet_type(),
            FIRMLogPacketType::BarometerPacket
        );
        assert_eq!(log_packet1.payload(), log_packet_bytes1[1..].as_ref());
        assert_eq!(log_packet2.payload(), log_packet_bytes2[1..].as_ref());

        let expected = 168.0f64 / 168e6;
        assert!((d2 - expected).abs() < 1e-12);
        assert!(parser.get_packet_and_time_delay().is_none());
    }

    #[test]
    fn test_split_bytes_and_garbage_resyncs() {
        let header = make_header();
        let log_packet_bytes = make_log_packet_bytes(MMC5983MA_ID, 0x123456, MMC5983MA_SIZE);

        let mut chunk1 = Vec::new();
        chunk1.push(0x99); // garbage byte
        chunk1.extend_from_slice(&log_packet_bytes[..5]);
        let chunk2 = &log_packet_bytes[5..];

        let mut parser = LogParser::new();
        parser.read_header(&header);

        parser.parse_bytes(&chunk1);
        assert!(parser.get_packet().is_none());

        parser.parse_bytes(chunk2);
        let log_packet = parser.get_packet().unwrap();
        assert_eq!(
            log_packet.packet_type(),
            FIRMLogPacketType::MagnetometerPacket
        );
        assert_eq!(
            log_packet.payload().len(),
            LOG_PACKET_TIMESTAMP_SIZE + MMC5983MA_SIZE
        );
        assert!(parser.get_packet().is_none());
    }
}
