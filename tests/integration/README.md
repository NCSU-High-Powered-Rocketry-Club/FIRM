# Integration Tests

These are the original python test suites covering several pipelines:

- the flight-data manager
- Mock USB, 
- web telemetry client.

## Commands

```bash
just test-python          # Fast unit + manager tests (skips @pytest.mark.integration)
just test-integration     # Full integration suite (Node + WASM + host C harness)

```

### Prerequisites for `@pytest.mark.integration`:

Integration tests require:

* Node.js
* Host C compiler
* Built TypeScript client:

```bash
cd client && npm ci && npm run build
```