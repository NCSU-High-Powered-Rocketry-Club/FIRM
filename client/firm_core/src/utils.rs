/// Converts a string to a fixed-size byte array, padding with zeros if necessary or
/// truncating if too long.
///
/// # Arguments
///
/// - `string` (`&str`) - The input string to convert.
///
/// # Returns
///
/// - `[u8; N]` - The fixed-size byte array representing the string, padded with zeros if necessary.
pub(crate) fn str_to_bytes<const N: usize>(string: &str) -> [u8; N] {
    let mut out = [0u8; N];
    let bytes = string.as_bytes();

    let n = bytes.len().min(N);
    out[..n].copy_from_slice(&bytes[..n]);
    out
}

/// Converts a fixed-size byte array to a string, stopping at the first zero byte.
///
/// # Arguments
///
/// - `bytes` (`&[u8]`) - The input byte slice to convert.
///
/// # Returns
///
/// - `String` - The resulting string, stopping at the first zero byte.
pub(crate) fn bytes_to_str(bytes: &[u8]) -> String {
    let mut end = bytes.len();
    for (i, &b) in bytes.iter().enumerate() {
        if b == 0 {
            end = i;
            break;
        }
    }
    String::from_utf8_lossy(&bytes[..end]).to_string()
}

pub(crate) fn parse_bytes_to_f32(bytes: &[u8], idx: &mut usize) -> f32 {
    assert!(bytes.len() >= 4, "Need at least 4 bytes to convert to f32");
    let value = f32::from_le_bytes(*bytes[*idx..].first_chunk().unwrap());
    *idx += 4;
    value
}

pub(crate) fn parse_bytes_to_many_f32s(bytes: &[u8], n: usize, idx: &mut usize) -> Vec<f32> {
    let mut values = Vec::with_capacity(n);
    for _ in 0..n {
        values.push(parse_bytes_to_f32(bytes, idx));
    }
    values
}
