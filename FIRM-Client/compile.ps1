# Will build for x86_64 and aarch64 Linux targets using Python 3.10 to 3.14 (including 3.14t):
uv run -p 3.14 -- maturin build --release -i 3.14t --compatibility pypi --target x86_64-unknown-linux-gnu --zig
uv run -p 3.14 -- maturin build --release -i 3.14 --compatibility pypi --target x86_64-unknown-linux-gnu --zig
uv run -p 3.14 -- maturin build --release -i 3.13 --compatibility pypi --target x86_64-unknown-linux-gnu --zig
uv run -p 3.14 -- maturin build --release -i 3.12 --compatibility pypi --target x86_64-unknown-linux-gnu --zig
uv run -p 3.14 -- maturin build --release -i 3.11 --compatibility pypi --target x86_64-unknown-linux-gnu --zig
uv run -p 3.14 -- maturin build --release -i 3.10 --compatibility pypi --target x86_64-unknown-linux-gnu --zig
uv run -p 3.14 -- maturin build --release -i 3.14t --compatibility pypi --target aarch64-unknown-linux-gnu --zig
uv run -p 3.14 -- maturin build --release -i 3.14 --compatibility pypi --target aarch64-unknown-linux-gnu --zig
uv run -p 3.14 -- maturin build --release -i 3.13 --compatibility pypi --target aarch64-unknown-linux-gnu --zig
uv run -p 3.14 -- maturin build --release -i 3.12 --compatibility pypi --target aarch64-unknown-linux-gnu --zig
uv run -p 3.14 -- maturin build --release -i 3.11 --compatibility pypi --target aarch64-unknown-linux-gnu --zig
uv run -p 3.14 -- maturin build --release -i 3.10 --compatibility pypi --target aarch64-unknown-linux-gnu --zig

# Will build for x86_64 Windows target using Python 3.10 to 3.14 (including 3.14t):
uv run -p 3.14 -- maturin build --release -i 3.14t --compatibility pypi --target x86_64-pc-windows-msvc
uv run -p 3.14 -- maturin build --release -i 3.14 --compatibility pypi --target x86_64-pc-windows-msvc
uv run -p 3.14 -- maturin build --release -i 3.13 --compatibility pypi --target x86_64-pc-windows-msvc
uv run -p 3.14 -- maturin build --release -i 3.12 --compatibility pypi --target x86_64-pc-windows-msvc
uv run -p 3.14 -- maturin build --release -i 3.11 --compatibility pypi --target x86_64-pc-windows-msvc
uv run -p 3.14 -- maturin build --release -i 3.10 --compatibility pypi --target x86_64-pc-windows-msvc