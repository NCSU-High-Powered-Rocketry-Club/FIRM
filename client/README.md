# FIRM Client

USB parser and client for FIRM, with Rust, Python, and WebAssembly bindings. This directory is a member of the repository Cargo workspace; run `cargo` from the **repository root**.

## Project Structure

- **`firm_core`**: The core `no_std` crate containing the raw USB message parser and data structures. This is the foundation for all other crates and can be used in embedded environments.
- **`firm_rust`**: A high-level Rust API that uses `serialport` to read from a serial device and provides a threaded client for receiving packets.
- **`firm_python`**: Python bindings for the Rust client (`firm-client`).
- **`firm_typescript`**: WebAssembly bindings and TypeScript code for using the parser in web applications.

## Philosophy

The goal is a single, efficient implementation of the FIRM parser across Rust, Python, Web/JS, and embedded.
By centralizing the parsing logic in `firm_core`, we ensure consistency and reduce code duplication.

## Building

### Prerequisites

- Rust (latest stable)
- Python 3.14+ (for Python bindings)
- `maturin` (for building Python wheels)
- `wasm-pack` (for building WASM)
- Node.js/npm (for TypeScript)

### Build Instructions

We assume that you are using a Unix-like environment (Linux or macOS).

Windows users may need to adapt some commands, or use WSL.

Make sure you have [Cargo](https://rustup.rs) and [uv](https://docs.astral.sh/uv/getting-started/installation/) installed.

You would also need npm if you want to test the web/TypeScript bindings.
Install it and Node.js here: https://nodejs.org/en/download/

From the repository root:

1.  Build host Rust crates:

    ```bash
    just build-rust
    # or: cargo build --workspace
    ```

2.  Build Python bindings:

    ```bash
    just sync
    ```

3.  Build WASM/TypeScript:

    ```bash
    cargo install wasm-pack
    rustup target add wasm32-unknown-unknown
    cd client
    npm install
    npm run build
    ```

## Running Tests

From the repository root:

```bash
just test-rust
# or: cargo test --workspace
```

## Usage

### Rust

Add `firm_rust` to your `Cargo.toml`.

```rust
use firm_rust::FIRMClient;
use std::{thread, time::Duration};

fn main() {
    let mut client = FIRMClient::new("/dev/ttyUSB0", 2_000_000, 0.1)
        .expect("failed to open serial port");
    client.start();

    loop {
        while let Ok(packets) = client.get_data_packets(Some(Duration::from_millis(100))) {
            for packet in packets {
                println!("{:#?}", packet);
            }
        }
    }
}
```

### Python

Install the published package from PyPI, or build it from source with `just sync` (see above).

```bash
pip install firm-client
```

This library supports Python 3.14 and above, including the free-threaded build (3.14t).

```python
from firm_client import FIRMClient

# Using context manager (automatically starts and stops)
with FIRMClient("/dev/ttyUSB0", baud_rate=2_000_000, timeout=0.1) as client:
    client.get_data_packets(block=True)  # Clear initial packets
    client.zero_out_pressure_altitude()
    while True:
        packets = client.get_data_packets()
        for packet in packets:
            print(packet.timestamp_seconds, packet.raw_acceleration_x_gs)
```

### Web (TypeScript)

```ts
import { FIRM } from 'firm-client';

const firm = await FIRM.connect();
for await (const packet of firm.getDataPackets()) {
  console.log(packet.timestamp_seconds, packet.raw_acceleration_x_gs);
}
```

## Publishing

This is mostly for maintainers, but here are the steps to publish each crate to their respective package registries:

### Rust API (crates.io)

These crates are not published to crates.io yet. Depend on them from this repository's Cargo workspace.

### Python Bindings (PyPI)

Wheels are built locally and then uploaded to PyPI. Each release covers Python 3.14 and 3.14t
(free-threaded) on Linux x86_64, Linux aarch64, and Windows x86_64. Run every command in this section
from the `client/` directory.

1. Always bump the version in `firm_python/Cargo.toml` before publishing.

2. Build the wheels:

```bash
# Linux or macOS (cross-compiles the Linux wheels with zig)
./compile.sh

# Windows
.\compile.ps1
```

This creates the wheels in the repo-root `target/wheels` directory.

3. Make sure you also have a source distribution:

```bash
uv run maturin sdist
```

4. We will use `uv` to publish these wheels to PyPI. Make sure you are part of the HPRC
   organization on PyPI, so you have access to the project and can publish new versions.

```bash
uv publish ../target/wheels/*
```

This will ask for PyPI credentials, make sure you get the token from the website.

### TypeScript Package (npm)

Run these from the `client/` directory. Building needs `wasm-pack` and the `wasm32-unknown-unknown`
Rust target (see the build instructions above).

1. Always bump the version in `firm_typescript/Cargo.toml` and `package.json` before publishing. Make sure they match.

2. Login to npm

`npm login`

3. Publish it

`npm publish`

## License

Licensed under the MIT License. See the repository `LICENSE` file for details.
