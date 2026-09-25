# Tests

| Path                 | How to run                                               |
|----------------------|----------------------------------------------------------|
| `tests/firmware/`    | `just test-firmware` (host CMake + Unity)                |
| `tests/integration/` | `just test-integration` (`pytest -m integration`)        |
| `tests/processing/`  | `just test-python` (ESKF lab and flight-data unit tests) |
| `tests/protocol/`    | `just test-host` (C header size and identifier checks)   |
| `client/tests/`      | `just test-python` (Python bindings for `firm-client`)   |
| Rust `#[cfg(test)]`  | `just test-rust`                                         |

`just test` runs firmware, host CTest, cargo, and pytest, then prints a pass/fail summary.
