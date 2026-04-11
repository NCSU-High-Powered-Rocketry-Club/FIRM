use firm_core::client_packets::{FIRMCommandPacket, FIRMLogPacket};
use firm_core::constants::command::{
    NUMBER_OF_CALIBRATION_OFFSETS, NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS,
};
use firm_core::constants::log_parsing::{FIRMLogPacketType, HEADER_TOTAL_SIZE};
use firm_core::data_parser::SerialParser;
use firm_core::firm_packets::{DeviceConfig, DeviceProtocol};
use firm_core::framed_packet::Framed;
use firm_core::log_parsing::LogParser;
use js_sys::{Object, Reflect, Uint8Array};
use serde::Serialize;
use wasm_bindgen::prelude::*;

use firm_core::calibration::MagnetometerCalibrator;
use firm_core::firm_packets::ProcessedFIRMData;

#[wasm_bindgen]
pub struct FIRMCommandBuilder;

#[wasm_bindgen]
impl FIRMCommandBuilder {
    pub fn build_get_device_info() -> Vec<u8> {
        FIRMCommandPacket::build_get_device_info_command().to_bytes()
    }

    pub fn build_get_device_config() -> Vec<u8> {
        FIRMCommandPacket::build_get_device_config_command().to_bytes()
    }

    pub fn build_get_calibration() -> Vec<u8> {
        FIRMCommandPacket::build_get_calibration_command().to_bytes()
    }

    pub fn build_set_device_config(
        name: String,
        frequency: u16,
        protocol: DeviceProtocol,
    ) -> Vec<u8> {
        let config = DeviceConfig {
            name,
            frequency,
            protocol,
        };

        FIRMCommandPacket::build_set_device_config_command(config).to_bytes()
    }

    pub fn build_set_imu_calibration(
        accel_offsets: Vec<f32>,
        accel_scale_matrix: Vec<f32>,
        gyro_offsets: Vec<f32>,
        gyro_scale_matrix: Vec<f32>,
    ) -> Vec<u8> {
        if accel_offsets.len() != NUMBER_OF_CALIBRATION_OFFSETS {
            wasm_bindgen::throw_str("accel_offsets must have length 3");
        }
        if accel_scale_matrix.len() != NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS {
            wasm_bindgen::throw_str("accel_scale_matrix must have length 9");
        }
        if gyro_offsets.len() != NUMBER_OF_CALIBRATION_OFFSETS {
            wasm_bindgen::throw_str("gyro_offsets must have length 3");
        }
        if gyro_scale_matrix.len() != NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS {
            wasm_bindgen::throw_str("gyro_scale_matrix must have length 9");
        }

        let accel_offsets_arr: [f32; NUMBER_OF_CALIBRATION_OFFSETS] =
            [accel_offsets[0], accel_offsets[1], accel_offsets[2]];
        let accel_scale_arr: [f32; NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS] = [
            accel_scale_matrix[0],
            accel_scale_matrix[1],
            accel_scale_matrix[2],
            accel_scale_matrix[3],
            accel_scale_matrix[4],
            accel_scale_matrix[5],
            accel_scale_matrix[6],
            accel_scale_matrix[7],
            accel_scale_matrix[8],
        ];

        let gyro_offsets_arr: [f32; NUMBER_OF_CALIBRATION_OFFSETS] =
            [gyro_offsets[0], gyro_offsets[1], gyro_offsets[2]];
        let gyro_scale_arr: [f32; NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS] = [
            gyro_scale_matrix[0],
            gyro_scale_matrix[1],
            gyro_scale_matrix[2],
            gyro_scale_matrix[3],
            gyro_scale_matrix[4],
            gyro_scale_matrix[5],
            gyro_scale_matrix[6],
            gyro_scale_matrix[7],
            gyro_scale_matrix[8],
        ];

        FIRMCommandPacket::build_set_imu_calibration_command(
            accel_offsets_arr,
            accel_scale_arr,
            gyro_offsets_arr,
            gyro_scale_arr,
        )
        .to_bytes()
    }

    pub fn build_set_magnetometer_calibration(
        offsets: Vec<f32>,
        scale_matrix: Vec<f32>,
    ) -> Vec<u8> {
        if offsets.len() != NUMBER_OF_CALIBRATION_OFFSETS {
            wasm_bindgen::throw_str("offsets must have length 3");
        }
        if scale_matrix.len() != NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS {
            wasm_bindgen::throw_str("scale_matrix must have length 9");
        }

        let offsets_arr: [f32; NUMBER_OF_CALIBRATION_OFFSETS] =
            [offsets[0], offsets[1], offsets[2]];
        let scale_arr: [f32; NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS] = [
            scale_matrix[0],
            scale_matrix[1],
            scale_matrix[2],
            scale_matrix[3],
            scale_matrix[4],
            scale_matrix[5],
            scale_matrix[6],
            scale_matrix[7],
            scale_matrix[8],
        ];

        FIRMCommandPacket::build_set_magnetometer_calibration_command(offsets_arr, scale_arr)
            .to_bytes()
    }

    pub fn build_cancel() -> Vec<u8> {
        FIRMCommandPacket::build_cancel_command().to_bytes()
    }

    pub fn build_reboot() -> Vec<u8> {
        FIRMCommandPacket::build_reboot_command().to_bytes()
    }

    pub fn build_mock() -> Vec<u8> {
        FIRMCommandPacket::build_mock_command().to_bytes()
    }
}

#[wasm_bindgen]
pub fn mock_header_size() -> usize {
    HEADER_TOTAL_SIZE
}

#[wasm_bindgen(js_name = FIRMDataParser)]
pub struct FIRMDataParser {
    inner: SerialParser,
}

impl Default for FIRMDataParser {
    fn default() -> Self {
        Self::new()
    }
}

#[wasm_bindgen(js_class = FIRMDataParser)]
impl FIRMDataParser {
    #[wasm_bindgen(constructor)]
    pub fn new() -> FIRMDataParser {
        FIRMDataParser {
            inner: SerialParser::new(),
        }
    }

    #[wasm_bindgen]
    pub fn parse_bytes(&mut self, data: &[u8]) {
        self.inner.parse_bytes(data);
    }

    #[wasm_bindgen]
    pub fn get_packet(&mut self) -> JsValue {
        match self.inner.get_data_packet() {
            Some(packet) => serde_wasm_bindgen::to_value(&packet).unwrap(),
            None => JsValue::NULL,
        }
    }

    #[wasm_bindgen]
    pub fn get_response(&mut self) -> JsValue {
        match self.inner.get_response_packet() {
            Some(frame) => serde_wasm_bindgen::to_value(frame.response()).unwrap(),
            None => JsValue::NULL,
        }
    }
}

#[wasm_bindgen(js_name = MockLogParser)]
pub struct MockLogParser {
    inner: LogParser,
}

impl Default for MockLogParser {
    fn default() -> Self {
        Self::new()
    }
}

#[wasm_bindgen(js_class = MockLogParser)]
impl MockLogParser {
    #[wasm_bindgen(constructor)]
    pub fn new() -> MockLogParser {
        MockLogParser {
            inner: LogParser::new(),
        }
    }

    #[wasm_bindgen]
    pub fn read_header(&mut self, header: &[u8]) {
        self.inner.read_header(header);
    }

    #[wasm_bindgen]
    pub fn parse_bytes(&mut self, data: &[u8]) {
        self.inner.parse_bytes(data);
    }

    #[wasm_bindgen]
    pub fn get_packet_with_delay(&mut self) -> JsValue {
        match self.inner.get_packet_and_time_delay() {
            Some((pkt, delay_seconds)) => {
                let bytes = pkt.to_bytes();
                let obj = Object::new();
                let _ = Reflect::set(
                    &obj,
                    &"bytes".into(),
                    &Uint8Array::from(bytes.as_slice()).into(),
                );
                let _ = Reflect::set(
                    &obj,
                    &"delaySeconds".into(),
                    &JsValue::from_f64(delay_seconds),
                );
                obj.into()
            }
            None => JsValue::NULL,
        }
    }

    #[wasm_bindgen]
    pub fn build_header_packet(&self, header: &[u8]) -> Vec<u8> {
        FIRMLogPacket::new(FIRMLogPacketType::HeaderPacket, header.to_vec()).to_bytes()
    }
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
struct MagnetometerCalibrationResult {
    offsets: [f32; 3],
    scale_matrix: [f32; 9],
    field_strength: f32,
    sample_count: usize,
}

/// WASM wrapper for magnetometer calibration.
///
/// Usage from JS/TS:
/// - `const cal = new MagnetometerCalibrator();`
/// - `cal.start();`
/// - `cal.add_sample(pkt);` (pkt is a parsed FIRMPacket / FIRMData object)
/// - `const res = cal.calculate();` (null if failed)
#[wasm_bindgen(js_name = MagnetometerCalibrator)]
pub struct MagnetometerCalibratorWasm {
    inner: MagnetometerCalibrator,
}

impl Default for MagnetometerCalibratorWasm {
    fn default() -> Self {
        Self::new()
    }
}

#[wasm_bindgen(js_class = MagnetometerCalibrator)]
impl MagnetometerCalibratorWasm {
    #[wasm_bindgen(constructor)]
    pub fn new() -> MagnetometerCalibratorWasm {
        MagnetometerCalibratorWasm {
            inner: MagnetometerCalibrator::new(),
        }
    }

    #[wasm_bindgen]
    pub fn start(&mut self) {
        self.inner.start();
    }

    #[wasm_bindgen]
    pub fn stop(&mut self) {
        self.inner.stop();
    }

    #[wasm_bindgen]
    pub fn sample_count(&self) -> usize {
        self.inner.sample_count()
    }

    /// Adds a sample from a parsed telemetry packet.
    ///
    /// Expects an object compatible with the processed telemetry serde shape.
    #[wasm_bindgen]
    pub fn add_sample(&mut self, packet: JsValue) {
        let data: ProcessedFIRMData = serde_wasm_bindgen::from_value(packet).unwrap_or_else(|e| {
            wasm_bindgen::throw_str(&format!("Failed to parse FIRMPacket for calibration: {e}"))
        });
        self.inner.add_sample(&data);
    }

    /// Calculates calibration parameters.
    ///
    /// Returns `null` if insufficient samples or solver failed.
    #[wasm_bindgen]
    pub fn calculate(&self) -> JsValue {
        match self.inner.calculate() {
            Some(cal) => {
                let (offsets, scale_matrix) = cal.to_arrays();
                let out = MagnetometerCalibrationResult {
                    offsets,
                    scale_matrix,
                    field_strength: cal.field_strength,
                    sample_count: self.inner.sample_count(),
                };
                serde_wasm_bindgen::to_value(&out).unwrap()
            }
            None => JsValue::NULL,
        }
    }
}
