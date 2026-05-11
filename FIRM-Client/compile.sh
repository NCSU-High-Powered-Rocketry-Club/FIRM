# Run this script to compile the library for different python versions and platforms:

# Will build for x86_64:
uv run -p 3.14 -- maturin build --release -i 3.14t --compatibility pypi
uv run -p 3.14 -- maturin build --release -i 3.14 --compatibility pypi
uv run -p 3.14 -- maturin build --release -i 3.13 --compatibility pypi
uv run -p 3.14 -- maturin build --release -i 3.12 --compatibility pypi
uv run -p 3.14 -- maturin build --release -i 3.11 --compatibility pypi
uv run -p 3.14 -- maturin build --release -i 3.10 --compatibility pypi

# Will build for aarch64:
uv run -p 3.14 -- maturin build --release -i 3.14t --compatibility pypi --target aarch64-unknown-linux-gnu --zig
uv run -p 3.14 -- maturin build --release -i 3.14 --compatibility pypi --target aarch64-unknown-linux-gnu --zig
uv run -p 3.14 -- maturin build --release -i 3.13 --compatibility pypi --target aarch64-unknown-linux-gnu --zig
uv run -p 3.14 -- maturin build --release -i 3.12 --compatibility pypi --target aarch64-unknown-linux-gnu --zig
uv run -p 3.14 -- maturin build --release -i 3.11 --compatibility pypi --target aarch64-unknown-linux-gnu --zig
uv run -p 3.14 -- maturin build --release -i 3.10 --compatibility pypi --target aarch64-unknown-linux-gnu --zig

uv run -p 3.14 -- maturin build --release -i 3.14t --compatibility pypi --target x86_64-pc-windows-msvc --zig
uv run -p 3.14 -- maturin build --release -i 3.14 --compatibility pypi --target x86_64-pc-windows-msvc --zig
uv run -p 3.14 -- maturin build --release -i 3.13 --compatibility pypi --target x86_64-pc-windows-msvc --zig
uv run -p 3.14 -- maturin build --release -i 3.12 --compatibility pypi --target x86_64-pc-windows-msvc --zig
uv run -p 3.14 -- maturin build --release -i 3.11 --compatibility pypi --target x86_64-pc-windows-msvc --zig
uv run -p 3.14 -- maturin build --release -i 3.10 --compatibility pypi --target x86_64-pc-windows-msvc --zig