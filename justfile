# Human and CI recipes. GitHub Actions calls these same recipes.

set dotenv-load := false

export CARGO_TERM_COLOR := "always"

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
    cargo build --workspace

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
test-firmware:
    cd STM32/tests && ceedling test:all

test-host: build-host
    ctest --preset host --output-on-failure

test-rust:
    cargo test --workspace

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
    cargo clippy --workspace -- -D warnings

# Same pinned clang-format and file list as the pre-commit hook (.pre-commit-config.yaml).
lint-clang:
    uv run pre-commit run clang-format --all-files --show-diff-on-failure

# Local coverage of the firmware, host, rust, python, and lint CI jobs.
# Skips integration (Node + wasm-pack); run `just test-integration` for that.
ci: build-firmware test-firmware test-host test-rust test-python lint
