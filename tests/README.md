# Tests

All tests that are not Ceedling are in here.

| Path                 | How to run                                               |
|----------------------|----------------------------------------------------------|
| `tests/integration/` | `just test-integration` (`pytest -m integration`)        |
| `tests/processing/`  | `just test-python` (ESKF lab and flight-data unit tests) |
| `tests/protocol/`    | `just test-host` (C header size and identifier checks)   |
| `client/tests/`      | `just test-python` (Python bindings for `firm-client`)   |
| `STM32/tests/`       | `just test-firmware` (`ceedling test:all`)               |
| Rust `#[cfg(test)]`  | `just test-rust`                                         |

`just test` runs firmware, host CTest, cargo, and pytest, then prints a pass/fail summary.
