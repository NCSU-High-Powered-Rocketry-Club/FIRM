# Integration tests

Pytest coverage for pipelines that span packages: flight-data manager, mocked USB, and the TypeScript client.

```bash
just test-python          # unit + manager tests (excludes @pytest.mark.integration)
just test-integration     # Node + WASM + host C harness
```

`@pytest.mark.integration` tests need Node.js, a host C compiler, and a built TypeScript client (`cd client && npm ci && npm run build`).
