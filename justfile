# Human and CI recipes. GitHub Actions calls these same recipes.

set dotenv-load := false

export CARGO_TERM_COLOR := "always"

# wasm-bindgen cdylib; host cargo test/clippy cannot build it.
cargo_host := "--workspace --exclude firm_typescript"

default:
    @just --list

# Install both Python packages into the root venv.
sync:
    uv sync --all-packages --extra usb --group dev

# Firmware ELF (ARM) + host ESKF + host cargo crates.
build: build-firmware build-host build-rust

build-firmware:
    cmake --preset firmware-debug
    cmake --build --preset firmware-debug

build-host:
    cmake --preset host
    cmake --build --preset host

build-rust:
    cargo build {{cargo_host}}

# Ceedling firmware tests + host CTest (ESKF + wire layout) + cargo + pytest.
test:
    #!/usr/bin/env bash
    set +e
    names=()
    codes=()
    run_suite() {
        local name="$1"
        shift
        echo
        echo "=== ${name} ==="
        "$@"
        local code=$?
        names+=("${name}")
        codes+=("${code}")
        return 0
    }
    run_suite firmware just test-firmware
    run_suite host just test-host
    run_suite rust just test-rust
    run_suite python just test-python
    echo
    echo "=== summary ==="
    failed=0
    for i in "${!names[@]}"; do
        if [[ "${codes[$i]}" -eq 0 ]]; then
            echo "PASS  ${names[$i]}"
        else
            echo "FAIL  ${names[$i]} (exit ${codes[$i]})"
            failed=1
        fi
    done
    exit "${failed}"

# Firmware Unity tests via Ceedling (Ruby >= 3.0, `gem install ceedling -v 1.0.1`).
[working-directory: 'STM32/tests']
test-firmware:
    ceedling test:all

test-host: build-host
    ctest --preset host --output-on-failure

test-rust:
    cargo test {{cargo_host}}

# Default markers only. Integration tests that need Node live on `test-integration`.
test-python:
    uv run pytest -m "not integration"

test-integration:
    uv run pytest -m integration

lint: lint-ruff lint-rust lint-clang

lint-ruff:
    uv run ruff check .
    uv run ruff format --check .

lint-rust:
    cargo fmt --all -- --check
    cargo clippy {{cargo_host}} -- -D warnings

# Host C sources added by this repo. STM32/Core is formatted by pre-commit on
# edit. A full-tree --Werror over Core/ is not a CI gate until that tree is
# clang-format clean.
lint-clang:
    clang-format --dry-run --Werror tests/protocol/firm_wire_layout_test.c processing/eskf_lab/native/src/*.c processing/eskf_lab/native/include/*.h processing/eskf_lab/native/tests/*.c

# Local coverage of the firmware, host, rust, python, and lint CI jobs.
# Skips integration (Node + wasm-pack); run `just test-integration` for that.
ci: build-firmware test-firmware test-host test-rust test-python lint
