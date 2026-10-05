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

# Every C test (CTest) + cargo + pytest.
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
    run_suite c just test-host
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

# Every C test: firmware unit tests, host ESKF, and wire layout (utest.h + CTest).
test-host: build-host
    ctest --preset host

# Only the firmware unit tests in STM32/tests.
test-firmware: build-host
    ctest --preset host -L firmware

test-rust:
    cargo test --workspace

# Default markers only. Integration tests that need Node live on `test-integration`.
test-python:
    uv run pytest -m "not integration"

test-integration: build-host
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

# Local coverage of the firmware, c-tests, rust, python, and lint CI jobs.
# Skips integration (Node + wasm-pack); run `just test-integration` for that.
ci: build-firmware test-host test-rust test-python lint

# Set `dry_run=true` to pack and check without uploading (e.g. `just dry_run=true publish`).
dry_run := "false"

_publish_flag := if dry_run == "true" { "--dry-run" } else { "" }

_cargo_publish_flag := if dry_run == "true" { "--dry-run --allow-dirty" } else { "" }

# Zig-cross the Windows wheels except when already building on Windows.
windows_wheel_zig := if os() == "windows" { "" } else { "--zig" }

# firm-client wheels (Linux/Windows, CPython + free-threaded 3.14) and sdist.
build-wheels:
    #!/usr/bin/env bash
    set -euo pipefail
    rm -rf target/wheels
    build() {
        local target="$1"
        shift
        uv run --directory client -p 3.14 -- maturin build --release -i 3.14t --compatibility pypi --target "${target}" "$@"
        uv run --directory client -p 3.14 -- maturin build --release -i 3.14 --compatibility pypi --target "${target}" "$@"
    }
    build x86_64-unknown-linux-gnu --zig
    build aarch64-unknown-linux-gnu --zig
    build x86_64-pc-windows-msvc {{ windows_wheel_zig }}
    uv run --directory client maturin sdist

# PyPI: firm-client. Bump `client/firm_python/Cargo.toml` first.
publish-pypi-client: build-wheels
    uv publish {{ _publish_flag }} "target/wheels/*"

# PyPI: firm-hprc. Bump `processing/pyproject.toml` first.
publish-pypi-hprc:
    uv build --package firm-hprc --out-dir target/pypi/firm-hprc --clear
    uv publish {{ _publish_flag }} "target/pypi/firm-hprc/*"

# PyPI: firm-client and firm-hprc.
publish-pypi: publish-pypi-client publish-pypi-hprc

# npm: firm-client (WASM + TypeScript). Bump `client/package.json` and `client/firm_typescript/Cargo.toml` first.
[working-directory('client')]
publish-npm:
    npm publish {{ _publish_flag }}

# crates.io: firm_core then firm_rust. Bump those crate versions first.
# firm_rust dry-run needs the matching firm_core version on crates.io already.
publish-crates:
    #!/usr/bin/env bash
    set -euo pipefail
    cargo publish -p firm_core {{ _cargo_publish_flag }}
    if [[ "{{ dry_run }}" == "true" ]]; then
        echo "Skipping firm_rust dry-run (registry resolve needs firm_core on crates.io)."
        exit 0
    fi
    cargo publish -p firm_rust

# PyPI + npm + crates.io. Needs HPRC registry credentials; prefer `just dry_run=true publish` first.
publish: publish-pypi publish-npm publish-crates
