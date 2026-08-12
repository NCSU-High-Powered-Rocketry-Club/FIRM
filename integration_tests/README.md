# Integration Tests

This folder contains pytest-based integration tests for the TypeScript -> FIRM Client -> mocked USB -> STM32 command pipeline.

The first test exercises `getDeviceInfo()` end to end:

1. TypeScript sends the command.
2. The FIRM Client writes the one-byte command ID over a mocked serial link.
3. A Python USB bridge forwards the command into a small host-built STM32 harness.
4. The STM32 `dispatch_command()` path builds the response.
5. The raw ID-plus-payload response returns to the TypeScript client and is asserted in pytest.

Run from the repo root with:

```bash
pytest integration_tests
```

Requirements:

- Python 3.10+
- `pytest`
- Node.js
- A C compiler such as `gcc` or `clang`
