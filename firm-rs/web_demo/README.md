# FIRM Web Demo

This is simple example of how to use the FIRM client library to communicate with FIRM over WebSerial. Unfouruatalely, WebSerial is only implemented in Chrome at the moment, so you have to use Chrome. WebAssembly (wasm) is used in order to run the Rust library in the browser.

## Build instructions

To build wasm, ensure you have the appropriate target installed:

```bash
rustup target add wasm32-unknown-unknown
```

Then build the library with the wasm feature enabled:

```bash
cargo build --target wasm32-unknown-unknown --release --features wasm
```

Next you must run wasm-bindgen on the produced .wasm file to generate the JS library

Install wasm-bindgen-cli if you don't have it:

```bash
cargo install -f wasm-bindgen-cli
```

Then run wasm-bindgen:

```bash
wasm-bindgen --out-dir web_demo/pkg --target web target/wasm32-unknown-unknown/release/firm_rs.wasm
```

Now that you have the library built, you can serve the website:

```bash
python -m http.server 8000 -d web_demo
```

or

```bash
npx serve web_demo
```

Open http://localhost:8000 in Chrome, plug in your FIRM board, and click the button to connect to the board. You should see the latest data displayed on the screen.
