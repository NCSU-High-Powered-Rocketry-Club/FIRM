# Human and CI recipes. GitHub Actions calls these same recipes.

set dotenv-load := false

# Windows has no `sh`; run line recipes in PowerShell. Shebang (bash) recipes still need Git Bash.
set windows-shell := ["powershell.exe", "-NoLogo", "-Command"]

export CARGO_TERM_COLOR := "always"

default:
    @just --list

# Install both Python packages into the root venv.
sync:
    uv sync --all-packages --extra usb --group dev

# Firmware ELF (ARM), native ESKF,  C tests, and cargo crates. ALL THE BUILDS
build: build-firmware build-native build-rust

# Debug firmware ELF for the STM32
build-firmware:
    cmake --preset firmware-debug
    cmake --build --preset firmware-debug

# Everything local to your computer.
build-native:
    cmake --preset native
    cmake --build --preset native

# Only the native ESKF library and `firm_eskf_replay`.
build-eskf:
    cmake --preset native
    cmake --build --preset native --target firm_eskf_replay

build-rust:
    cargo build --workspace

# Node deps for the client/ + web/ npm workspace. Needs Node and wasm-pack (client `prepare` builds WASM).
sync-web:
    npm ci

# React web app. Run `just sync-web` first.
build-web:
    npm run build -w web

web-dev:
    npm run dev -w web

build-docs:
    uv run --group docs mkdocs build

# Every C test (CTest) + cargo + pytest, excluding integration.
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
    run_suite c just test-c
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

# Every C test (firmware unit tests, ESKF, and wire layout).
test-c: build-native
    ctest --preset native

# Only the firmware unit tests in STM32/tests.
test-firmware: build-native
    ctest --preset native -L firmware

test-rust:
    cargo test --workspace

# Default markers only, non-integration
test-python:
    uv run pytest -m "not integration"

test-integration: build-native sync-web
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

# ESLint, Prettier, and tsc over client/ and web/. Needs `just sync-web`; not part of `just lint` so firmware/Python checkouts stay node-free.
lint-web:
    npm run lint
    npm run format:check
    npm run typecheck

# Local coverage of the firmware, c-tests, rust, python, and lint CI jobs.
# Skips the Node jobs: run `just test-integration` and `just sync-web lint-web build-web build-docs` for those.
ci: build-firmware test-c test-rust test-python lint

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
publish-npm: sync-web
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

# Publish to PyPI, npm, and crates.io. Needs credentials; you should run `just dry_run=true publish` to make sure everything is OK.
publish: publish-pypi publish-npm publish-crates
