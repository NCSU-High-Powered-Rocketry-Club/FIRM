# Integration Tests

End-to-end pytest suites:

- `test_flight_data_manager.py`: ingest, migration, trimming, and staleness of the flight-log
  archive on generated logs. Most cases run under `just test-python`;
  `test_manager_output_is_accepted_by_rust_playback` is marked `integration`.
- `test_get_device_info_pipeline.py`: the Web/WASM client talking to the real firmware command
  dispatch, compiled into `stm32_device_info_harness.c`.
- `test_web_usb_telemetry_pipeline.py`: the browser client parsing raw STM32 USB telemetry.

## Commands

```bash
just test-python          # Fast unit + manager tests (skips @pytest.mark.integration)
just test-integration     # Full integration suite (Node + WASM + host C harness)
```

### Prerequisites for `@pytest.mark.integration`

- Node.js
- Host C toolchain (the harness is built by `just build-host`, which `just test-integration` runs)
- Built TypeScript client:

```bash
cd client && npm ci
```
