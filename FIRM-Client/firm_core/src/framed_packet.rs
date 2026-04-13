use alloc::vec::Vec;

use crate::{constants::packet::*, utils::crc16_ccitt};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FrameError {
    TooShort,
    LengthMismatch { expected: usize, got: usize },
    BadCrc { expected: u16, got: u16 },
    UnknownIdentifier(u16),
}

/// Trait implemented by all packet types that are framed using FramedPacket.
pub trait Framed: Sized {
    fn frame(&self) -> &FramedPacket;

    fn from_bytes(bytes: &[u8]) -> Result<Self, FrameError>;

    fn header(&self) -> PacketHeader {
        self.frame().header()
    }

    fn identifier(&self) -> u16 {
        self.frame().identifier()
    }

    fn payload(&self) -> &[u8] {
        self.frame().payload()
    }

    fn len(&self) -> u32 {
        self.frame().len()
    }

    fn is_empty(&self) -> bool {
        self.frame().is_empty()
    }

    fn crc(&self) -> u16 {
        self.frame().crc()
    }

    fn to_bytes(&self) -> Vec<u8> {
        self.frame().to_bytes()
    }
}

/// Shared packet framing for the wire format:
/// `[header(2)][identifier(2)][length(4)][payload(len)][crc(2)]`.
///
/// CRC is computed over everything before the CRC: `header + identifier + len + payload`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FramedPacket {
    header: PacketHeader,
    identifier: u16,
    // There is also a length field in the header, but we store it implicitly as the length of the payload.
    payload: Vec<u8>,
    crc: u16,
}

impl FramedPacket {
    pub const MIN_SIZE: usize = HEADER_SIZE + IDENTIFIER_SIZE + LENGTH_SIZE + CRC_SIZE;

    pub fn new(header: PacketHeader, identifier: u16, payload: Vec<u8>) -> Self {
        let crc = Self::compute_crc(header, identifier, payload.len() as u32, &payload);
        Self {
            header,
            identifier,
            payload,
            crc,
        }
    }

    pub fn header(&self) -> PacketHeader {
        self.header
    }

    pub fn identifier(&self) -> u16 {
        self.identifier
    }

    pub fn payload(&self) -> &[u8] {
        &self.payload
    }

    pub fn crc(&self) -> u16 {
        self.crc
    }

    pub fn len(&self) -> u32 {
        self.payload.len() as u32
    }

    pub fn is_empty(&self) -> bool {
        self.payload.is_empty()
    }

    pub fn to_bytes(&self) -> Vec<u8> {
        let len = self.payload.len() as u32;
        let mut out = Vec::with_capacity(
            HEADER_SIZE + IDENTIFIER_SIZE + LENGTH_SIZE + self.payload.len() + CRC_SIZE,
        );
        out.extend_from_slice(&self.header.as_u16().to_le_bytes());
        out.extend_from_slice(&self.identifier.to_le_bytes());
        out.extend_from_slice(&len.to_le_bytes());
        out.extend_from_slice(&self.payload);
        out.extend_from_slice(&self.crc.to_le_bytes());
        out
    }

    /// Parses a single framed packet from `bytes`, requiring that `bytes` contains
    /// exactly one full frame.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, FrameError> {
        if bytes.len() < Self::MIN_SIZE {
            return Err(FrameError::TooShort);
        }

        let header_raw = u16::from_le_bytes(bytes[0..HEADER_SIZE].try_into().unwrap());
        let header =
            PacketHeader::from_u16(header_raw).ok_or(FrameError::UnknownIdentifier(header_raw))?;
        let identifier = u16::from_le_bytes(
            bytes[HEADER_SIZE..HEADER_SIZE + IDENTIFIER_SIZE]
                .try_into()
                .unwrap(),
        );

        let len_start = HEADER_SIZE + IDENTIFIER_SIZE;
        let len = u32::from_le_bytes(
            bytes[len_start..len_start + LENGTH_SIZE]
                .try_into()
                .unwrap(),
        ) as usize;

        let expected = HEADER_SIZE + IDENTIFIER_SIZE + LENGTH_SIZE + len + CRC_SIZE;
        if bytes.len() != expected {
            return Err(FrameError::LengthMismatch {
                expected,
                got: bytes.len(),
            });
        }

        let payload_start = HEADER_SIZE + IDENTIFIER_SIZE + LENGTH_SIZE;
        let payload_end = payload_start + len;
        let payload = bytes[payload_start..payload_end].to_vec();

        let received_crc = u16::from_le_bytes(
            bytes[payload_end..payload_end + CRC_SIZE]
                .try_into()
                .unwrap(),
        );
        let computed_crc = Self::compute_crc(header, identifier, len as u32, &payload);
        if received_crc != computed_crc {
            return Err(FrameError::BadCrc {
                expected: computed_crc,
                got: received_crc,
            });
        }

        Ok(Self {
            header,
            identifier,
            payload,
            crc: received_crc,
        })
    }

    /// Computes CRC over `[header][identifier][length][payload]`.
    pub fn compute_crc(header: PacketHeader, identifier: u16, len: u32, payload: &[u8]) -> u16 {
        let mut crc_input =
            Vec::with_capacity(HEADER_SIZE + IDENTIFIER_SIZE + LENGTH_SIZE + payload.len());
        crc_input.extend_from_slice(&header.as_u16().to_le_bytes());
        crc_input.extend_from_slice(&identifier.to_le_bytes());
        crc_input.extend_from_slice(&len.to_le_bytes());
        crc_input.extend_from_slice(payload);
        crc16_ccitt(&crc_input)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn framed_packet_roundtrip() {
        let header = PacketHeader::Data;
        let identifier = 0x0000u16;
        let payload = vec![1u8, 2, 3, 4, 5];
        let pkt = FramedPacket::new(header, identifier, payload.clone());
        let bytes = pkt.to_bytes();

        let parsed = FramedPacket::from_bytes(&bytes).unwrap();
        assert_eq!(parsed.header(), header);
        assert_eq!(parsed.identifier(), identifier);
        assert_eq!(parsed.payload(), payload.as_slice());
        assert_eq!(parsed.crc(), pkt.crc());
    }
}
