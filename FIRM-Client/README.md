# FIRM-Client

A modular Rust library for parsing FIRM data packets, with bindings for Python and WebAssembly.

## Project Structure

The project is organized as a Cargo workspace with the following crates:

- **`firm_core`**: The core `no_std` crate containing the packet parser, CRC logic, and data structures. This is the foundation for all other crates and can be used in embedded environments.
- **`firm_rust`**: A high-level Rust API that uses `serialport` to read from a serial device and provides a threaded client for receiving packets.
- **`firm_python`**: Python bindings for the Rust client.
- **`firm_typescript`**: WebAssembly bindings and TypeScript code for using the parser in web applications.

## Philosophy

The goal of FIRM-Client is to provide a single, efficient, and correct implementation of the FIRM parser that can be used across different ecosystems (Rust, Python, Web/JS, Embedded).
By centralizing the parsing logic in `firm_core`, we ensure consistency and reduce code duplication.

## Building

### Prerequisites

- Rust (latest stable)
- Python 3.10+ (for Python bindings)
- `maturin` (for building Python wheels)
- `wasm-pack` (for building WASM)
- Node.js/npm (for TypeScript)

### Build Instructions

We assume that you are using a Unix-like environment (Linux or macOS).

Windows users may need to adapt some commands (we will mention where this is the case), or use
WSL (Windows Subsystem for Linux) for best compatibility.

Make sure you have [Cargo](https://rustup.rs) and [uv](https://docs.astral.sh/uv/getting-started/installation/) installed.

You would also need npm if you want to test the web/TypeScript bindings.
Install it and Node.js here: https://nodejs.org/en/download/

1.  **Build all Rust crates:**

    ```bash
    cargo build
    ```

2.  **Build Python bindings:**

    ```bash
    cargo build -p firm_python
    uv sync
    # or to build a wheel
    uv run maturin build --release
    ```

3.  **Build WASM/TypeScript:**

    ```bash
    cd firm_typescript
    npm install
    npm run clean
    npm run build

    # For testing the code with examples/index.html
    npx serve .
    ```

## Usage

### Rust

Add `firm_rust` to your `Cargo.toml`.

```rust
use firm_rust::FirmClient;
use std::{thread, time::Duration};

fn main() {
    let mut client = FIRMClient::new("/dev/ttyUSB0", 2_000_000, 0.1);
    client.start();

    loop {
        while let Ok(packet) = client.get_packets(Some(Duration::from_millis(100))) {
            println!("{:#?}", packet);
        }
    }
}
```

### Python

You can install the library via pip (once published) or build from source.

```bash
pip install firm-client
```

This library supports Python 3.10 and above, including Python 3.14 free threaded.

```python
from firm_client import FIRMClient
import time

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

todo: Add usage example.

## Publishing

This is mostly for maintainers, but here are the steps to publish each crate to their respective package registries:

### Rust API (crates.io)

todo (idk actually know yet)

### Python Bindings (PyPI)

We need to to first build wheels for each platform, right now the workflow is to do this locally
and then upload to PyPI. At the minimum, we build for Linux x86_64 and aarch64 for python versions
3.10+, including free threaded wheels.

1. Always bump the version in `firm_python/Cargo.toml` before publishing.

2. Build the wheels 
```bash
# If you're on Linux
./compile.sh

# If you're on Windows
.\compile.ps1
```

This will create wheels in the `target/wheels` directory, for Python versions 3.10 to 3.14,
for both x86_64 and aarch64.

3. Make sure you also have a source distribution:

```bash
uv run maturin sdist
```

4. We will use `uv` to publish these wheels to PyPI. Make sure you are part of the HPRC
   organization on PyPI, so you have access to the project and can publish new versions.

```bash
uv publish target/wheels/*
```

This will ask for PyPI credentials, make sure you get the token from the website.

### TypeScript Package (npm)

1. Always bump the version in `firm_typescript/Cargo.toml` and `package.json` before publishing. Make sure they match.

2. Login to npm

`npm login`

3. Publish it

`npm publish`

(Ensure the version in `package.json` is bumped before publishing.)

## License

Licensed under the MIT License. See `LICENSE` file for details.
