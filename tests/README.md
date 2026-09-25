# Tests

| Path                 | How to run                                               |
|----------------------|----------------------------------------------------------|
| `STM32/tests/`       | `just test-firmware` (CTest label `firmware`)            |
| `tests/integration/` | `just test-integration` (`pytest -m integration`)        |
| `tests/processing/`  | `just test-python` (ESKF lab and flight-data unit tests) |
| `tests/protocol/`    | `just test-host` (C header size and identifier checks)   |
| `client/tests/`      | `just test-python` (Python bindings for `firm-client`)   |
| Rust `#[cfg(test)]`  | `just test-rust`                                         |

`just test` runs every C test, cargo, and pytest, then prints a pass/fail summary.

All C tests use [utest.h](../third_party/utest) and are plain CTest executables:
`just test-host` (or `ctest --preset host`) runs them all, and `ctest --preset host -L <label>`
runs one group (`firmware`, `eskf`, `protocol`). To add one, write `UTEST(...)` cases ending in
`UTEST_MAIN()` and register the file with `firm_add_c_test` in the nearest `CMakeLists.txt`.
