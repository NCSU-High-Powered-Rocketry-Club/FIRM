use firm_core::client_packets::FIRMCommandPacket;
use firm_core::constants::command::FIRMCommand;
use firm_core::data_parser::SerialParser;
use firm_core::firm_packets::{DeviceConfig, DeviceProtocol};

// These values and layouts deliberately come from the STM32 source of truth:
// Core/Inc/shared_data/identifiers.h, Core/Inc/shared_data/system_settings.h,
// Core/Src/tasks/usb_read_data_task.c, and Core/Src/tasks/packetizer_task.c.

#[test]
fn command_identifiers_match_stm32_identifiers() {
    let expected = [
        (FIRMCommand::GetDeviceInfo, 0x02),
        (FIRMCommand::GetDeviceConfig, 0x03),
        (FIRMCommand::SetDeviceConfig, 0x04),
        (FIRMCommand::Reboot, 0x05),
        (FIRMCommand::Mock, 0x06),
        (FIRMCommand::SetMagnetometerCalibration, 0x07),
        (FIRMCommand::SetIMUCalibration, 0x08),
        (FIRMCommand::GetCalibration, 0x09),
        (FIRMCommand::Cancel, 0x0A),
    ];

    let mismatches: Vec<_> = expected
        .into_iter()
        .filter(|(command, stm32_id)| command.to_u8() != *stm32_id)
        .map(|(command, stm32_id)| (command, command.to_u8(), stm32_id))
        .collect();

    assert!(
        mismatches.is_empty(),
        "Rust command IDs do not match STM32 (command, rust_id, stm32_id): {mismatches:?}"
    );
}

#[test]
fn get_device_info_command_matches_stm32_usb_wire_format() {
    // The STM32 receive task reads one identifier byte and then the identifier-specific
    // payload. GET_DEVICE_INFO has no payload, so its complete USB message is one byte.
    let expected_stm32_message = [0x02];
    let rust_message = FIRMCommandPacket::build_get_device_info_command().to_bytes();

    assert_eq!(
        rust_message, expected_stm32_message,
        "the Rust client must send the raw STM32 command, not an additional framed protocol"
    );
}

#[test]
fn set_device_config_payload_matches_stm32_device_config_layout() {
    let config = DeviceConfig {
        name: "FIRM".to_string(),
        frequency: 100,
        protocol: DeviceProtocol::UART,
    };
    let packet = FIRMCommandPacket::build_set_device_config_command(config);

    // STM32 DeviceConfig_t is:
    // [frequency_hz: u16][device_name: char[32]][usb][uart][i2c][spi].
    let mut expected_payload = Vec::new();
    expected_payload.extend_from_slice(&100u16.to_le_bytes());
    let mut name = [0u8; 32];
    name[..4].copy_from_slice(b"FIRM");
    expected_payload.extend_from_slice(&name);
    expected_payload.extend_from_slice(&[1, 1, 0, 0]);

    assert_eq!(
        packet.payload(),
        expected_payload,
        "Rust DeviceConfig payload layout must match the STM32 C struct"
    );
}

#[test]
fn serial_parser_accepts_raw_stm32_data_packet() {
    // STM32 sends [ID_DATA_PACKET: u8][DataPacket_t bytes] directly through CDC.
    // DataPacket_t is one f64 followed by twenty f32 values (88 payload bytes).
    let mut message = vec![0x01];
    let mut payload = vec![0u8; 88];
    payload[0..8].copy_from_slice(&42.0f64.to_le_bytes());
    payload[8..12].copy_from_slice(&25.0f32.to_le_bytes());
    payload[12..16].copy_from_slice(&101_325.0f32.to_le_bytes());
    payload[72..76].copy_from_slice(&1.0f32.to_le_bytes()); // quaternion W
    message.extend_from_slice(&payload);

    let mut parser = SerialParser::new();
    parser.parse_bytes(&message);
    payload[0..8].copy_from_slice(&43.0f64.to_le_bytes());
    let mut following_message = vec![0x01];
    following_message.extend_from_slice(&payload);
    parser.parse_bytes(&following_message);

    let parsed = parser
        .get_data_packet()
        .expect("the Rust parser discarded a valid raw STM32 telemetry message");
    assert_eq!(parsed.timestamp_seconds, 42.0);
    assert_eq!(parsed.temperature_celsius, 25.0);
    assert_eq!(parsed.pressure_pascals, 101_325.0);
}
