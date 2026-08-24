#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PacketError {
    TooShort,
    LengthMismatch { expected: usize, got: usize },
    UnknownIdentifier(u8),
    UnexpectedIdentifier { expected: u8, got: u8 },
    InvalidPayload(u8),
}

impl core::fmt::Display for PacketError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(formatter, "{self:?}")
    }
}
