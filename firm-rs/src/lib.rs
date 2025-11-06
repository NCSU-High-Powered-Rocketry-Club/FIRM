pub mod crc;
pub mod parser;

// wasm-bindgen bindings
#[cfg(feature = "wasm")]
pub mod wasm_bindings {
    use super::parser::SerialParser;
    use serde_wasm_bindgen::to_value;
    use wasm_bindgen::prelude::*;

    #[wasm_bindgen(js_name = FIRM)]
    pub struct WasmSerialParser {
        inner: SerialParser,
    }

    #[wasm_bindgen(js_class = FIRM)]
    impl WasmSerialParser {
        #[wasm_bindgen(constructor)]
        pub fn new() -> WasmSerialParser {
            WasmSerialParser {
                inner: SerialParser::new(),
            }
        }

        pub fn parse_bytes(&mut self, data: &[u8]) {
            self.inner.parse_bytes(data);
        }

        pub fn get_packet(&mut self) -> JsValue {
            match self.inner.get_packet() {
                Some(packet) => to_value(&packet).unwrap_or(JsValue::NULL),
                None => JsValue::NULL,
            }
        }
    }
}
