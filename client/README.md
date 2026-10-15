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
- Node.js/npm (for TypeScript bindings)

### Build Instructions

Development commands below assume you are using either a Linux or macOS environment. Windows users can run commands in PowerShell or use WSL.

Prerequisites: [Cargo](https://rustup.rs), [uv](https://docs.astral.sh/uv/getting-started/installation/), and [Node.js / npm](https://nodejs.org/en/download/) (for Web/TypeScript bindings).

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
    npm ci
    ```

    Note that `npm ci` runs the `prepare` script, which compiles both the WASM package and the TypeScript sources.

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
use std::time::Duration;

fn main() {
    let mut client = FIRMClient::new("/dev/ttyUSB0", 2_000_000, 0.1)
        .expect("failed to open serial port");
    client.start();

    loop {
        match client.get_data_packets(Some(Duration::from_millis(100))) {
            Ok(packets) => {
                for packet in packets {
                    println!("{:#?}", packet);
                }
            }
            Err(e) => {
                eprintln!("Error receiving packets: {e}");
                std::thread::sleep(Duration::from_millis(100));
            }
        }
    }
}
```

### Python

Install the published package from PyPI, or build it locally with `just sync` (see above). Pre-built wheels cover Linux and Windows; macOS installs compile from source (sdist) and require a local Rust toolchain.

```bash
pip install firm-client
```

This library supports Python 3.14 and above, including the free-threaded build (3.14t).

```python
from firm_client import FIRMClient

# Using context manager (automatically starts and stops)
with FIRMClient("/dev/ttyUSB0", baud_rate=2_000_000, timeout=0.1) as client:
    client.get_data_packets(block=True)  # Clear initial packets
    while True:
        packets = client.get_data_packets()
        for packet in packets:
            print(packet.timestamp_seconds, packet.raw_acceleration_x_gs)
```

### Web (TypeScript)

The browser client connects via the Web Serial API (supported in Chromium-based desktop browsers such as Chrome or Edge over HTTPS or localhost):

```ts
import { FIRM } from 'firm-client';

const firm = await FIRM.connect();
for await (const packet of firm.getDataPackets()) {
  console.log(packet.timestamp_seconds, packet.raw_acceleration_x_gs);
}
```

## Publishing

From the repository root. Preview with `just dry_run=true <recipe>` (no upload). `just publish` also publishes `firm-hprc` to PyPI.

| Registry   | Package                         | Bump first                                                                 | Recipe                     |
|------------|---------------------------------|----------------------------------------------------------------------------|----------------------------|
| crates.io  | `firm_core`, `firm_rust`        | `firm_core/Cargo.toml`, `firm_rust/Cargo.toml` (keep the path dep version in sync) | `just publish-crates`      |
| PyPI       | `firm-client`                   | `firm_python/Cargo.toml`                                                   | `just publish-pypi-client` |
| npm        | `firm-client`                   | `firm_typescript/Cargo.toml` and `package.json` (keep them equal)          | `just publish-npm`         |

Wheels are Zig-cross-compiled for Python 3.14 and 3.14t on Linux x86_64, Linux aarch64, and Windows x86_64; macOS installs build from the sdist. Publishing needs HPRC maintainer credentials on each registry (`UV_PUBLISH_TOKEN`, `npm login`, `cargo login`). `just publish-crates` uploads `firm_core` first; wait for crates.io to index it if `firm_rust` cannot see the new version yet.

## License

Licensed under the MIT License. See the repository `LICENSE` file for details.
