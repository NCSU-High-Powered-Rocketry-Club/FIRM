"""End-to-end STM32 C -> Web/WASM client device-info test."""

# Assertions, local subprocesses, and descriptive exception messages are intentional here.
# ruff: noqa: S603, TRY003

from __future__ import annotations

import json
import shutil
import socket
import struct
import subprocess
import textwrap
import threading
from dataclasses import dataclass
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
FIRM_CLIENT_ROOT = REPO_ROOT / "client"
TYPESCRIPT_DIST_INDEX = FIRM_CLIENT_ROOT / "firm_typescript" / "typescript" / "dist" / "index.js"

GET_DEVICE_INFO_ID = 0x02
# Must match kSettings in stm32_device_info_harness.c.
DEVICE_UID = 0x1122334455667788
FIRMWARE_VERSION = "v2.2.0"


def find_stm32_harness() -> Path | None:
    """Return the CMake-built command-dispatch harness (`just build-host`)."""
    bin_dir = REPO_ROOT / "build" / "host" / "bin"
    for directory in (bin_dir, bin_dir / "Release"):
        for name in ("firm_device_info_harness", "firm_device_info_harness.exe"):
            if (directory / name).exists():
                return directory / name
    return None


@dataclass
class BridgeResult:
    """Raw messages captured at the Web Serial/C firmware boundary."""

    command_message: bytes
    response_message: bytes


class RawUsbBridge:
    """Forward bytes unchanged between a Web Serial test double and the C harness."""

    def __init__(self, harness_exe: Path) -> None:
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(("127.0.0.1", 0))
        self._server.listen(1)
        self.port = self._server.getsockname()[1]
        self.result: BridgeResult | None = None
        self._harness = subprocess.Popen(
            [str(harness_exe)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def serve_once(self) -> None:
        """Forward one command and its response without protocol translation."""
        conn, _ = self._server.accept()
        with conn:
            command = conn.recv(4096)
            if not command:
                return
            response = self._send_to_harness(command)
            self.result = BridgeResult(command, response)
            conn.sendall(response)

    def _send_to_harness(self, command: bytes) -> bytes:
        if self._harness.stdin is None or self._harness.stdout is None:
            raise RuntimeError("Harness pipes are unavailable")
        self._harness.stdin.write(struct.pack("<I", len(command)))
        self._harness.stdin.write(command)
        self._harness.stdin.flush()
        length_bytes = self._harness.stdout.read(4)
        if len(length_bytes) != 4:
            raise RuntimeError("Harness did not return a response length")
        length = struct.unpack("<I", length_bytes)[0]
        response = self._harness.stdout.read(length)
        if len(response) != length:
            raise RuntimeError("Harness response was truncated")
        return response

    def close(self) -> None:
        """Close the test bridge and child process."""
        self._server.close()
        if self._harness.poll() is None:
            self._harness.terminate()
            self._harness.wait(timeout=5)


def make_node_script(script_path: Path, bridge_port: int) -> None:
    """Create a Node Web Serial test client using the built package."""
    script_path.write_text(
        textwrap.dedent(
            f"""
            import net from 'node:net';
            import {{ once }} from 'node:events';
            import {{ readFile }} from 'node:fs/promises';
            import {{ FIRM }} from {json.dumps(TYPESCRIPT_DIST_INDEX.as_uri())};

            Object.defineProperty(globalThis, 'navigator', {{
              value: {{ serial: {{}} }}, configurable: true,
            }});
            const originalFetch = globalThis.fetch?.bind(globalThis);
            globalThis.fetch = async (input, init) => {{
              const url = input instanceof URL
                ? input
                : (typeof input === 'string' ? new URL(input) : null);
              if (url?.protocol === 'file:') {{
                return new Response(await readFile(url), {{
                  headers: {{ 'Content-Type': 'application/wasm' }},
                }});
              }}
              return originalFetch(input, init);
            }};

            class SocketSerialPort {{
              async open() {{
                this.socket = net.createConnection({{ host: '127.0.0.1', port: {bridge_port} }});
                this.socket.setNoDelay(true);
                await once(this.socket, 'connect');
                const socket = this.socket;
                this.readable = new ReadableStream({{
                  start(controller) {{
                    socket.on('data', chunk => controller.enqueue(new Uint8Array(chunk)));
                    socket.on('error', error => controller.error(error));
                  }},
                  cancel() {{ socket.destroy(); }},
                }});
                this.writable = new WritableStream({{
                  write(chunk) {{
                    return new Promise((resolve, reject) =>
                      socket.write(Buffer.from(chunk), error => error ? reject(error) : resolve()));
                  }},
                  close() {{ socket.end(); }},
                }});
              }}
              async close() {{ this.socket?.destroy(); }}
            }}

            const firm = await FIRM.connect({{ baudRate: 115200, port: new SocketSerialPort() }});
            console.log(JSON.stringify(await firm.getDeviceInfo()));
            await firm.close();
            """
        ),
        encoding="utf-8",
    )


@pytest.mark.integration
def test_get_device_info_pipeline(tmp_path: Path) -> None:
    """Send and receive the raw one-ID-byte protocol without a translating bridge."""
    if not TYPESCRIPT_DIST_INDEX.exists():
        pytest.skip("TypeScript client dist is missing; build the client first.")
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node.js is required for the Web Serial integration test.")

    harness = find_stm32_harness()
    if harness is None:
        pytest.skip("STM32 device-info harness is missing; run `just build-host` first.")

    bridge = RawUsbBridge(harness)
    script = tmp_path / "device_info_pipeline.mjs"
    make_node_script(script, bridge.port)
    thread = threading.Thread(target=bridge.serve_once, daemon=True)
    thread.start()
    result = subprocess.run(
        [node, str(script)],
        check=False,
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
    )
    thread.join(timeout=5)
    bridge.close()

    assert result.returncode == 0, f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    assert bridge.result is not None
    assert bridge.result.command_message == bytes([GET_DEVICE_INFO_ID])
    assert bridge.result.response_message[0] == GET_DEVICE_INFO_ID
    assert len(bridge.result.response_message) == 1 + 8 + 8
    assert json.loads(result.stdout) == {
        "id": str(DEVICE_UID),
        "firmware_version": FIRMWARE_VERSION,
    }
