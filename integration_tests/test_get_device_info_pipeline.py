"""End-to-end STM32 C -> Web/WASM client device-info test."""

# Assertions, local subprocesses, and descriptive exception messages are intentional here.
# ruff: noqa: S101, S603, TRY003

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

REPO_ROOT = Path(__file__).resolve().parents[1]
FIRM_CLIENT_ROOT = REPO_ROOT / "FIRM-Client"
STM32_CORE_ROOT = REPO_ROOT / "STM32" / "Core"
TYPESCRIPT_DIST_INDEX = FIRM_CLIENT_ROOT / "firm_typescript" / "typescript" / "dist" / "index.js"

GET_DEVICE_INFO_ID = 0x02
DEVICE_UID = 0x1122334455667788
FIRMWARE_VERSION = "v2.2.0"


def compile_stm32_harness(build_dir: Path) -> Path:
    """Compile real STM32 command dispatch with host-side dependency stubs."""
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        raise RuntimeError("No C compiler found. Install gcc or clang to run the integration test.")

    harness_c = build_dir / "stm32_device_info_harness.c"
    harness_exe = build_dir / "stm32_device_info_harness.exe"
    harness_c.write_text(
        textwrap.dedent(
            f"""
            #include "commands.h"
            #include "adxl371_packet.h"
            #include "bmp581_packet.h"
            #include "icm45686_packet.h"
            #include "mmc5983ma_packet.h"
            #include "modules/transmit_frame.h"
            #include "shared_data/system_settings.h"

            #include <stdbool.h>
            #include <stdint.h>
            #include <stdio.h>
            #include <string.h>

            static const SystemSettings_t kSettings = {{
              .device_uid = {DEVICE_UID}ULL,
              .device_name = "INTEGRATION_TEST_DEVICE",
              .usb_transfer_enabled = true,
              .firmware_version = "{FIRMWARE_VERSION}",
              .frequency_hz = 100U,
            }};

            static uint8_t response[256];
            static uint32_t response_len;

            const SystemSettings_t *get_settings(void) {{ return &kSettings; }}
            int settings_write_firm_settings(SystemSettings_t *settings) {{
              (void)settings;
              return 0;
            }}
            int settings_write_calibration(Calibration_t *accel, Calibration_t *gyro,
                                           Calibration_t *mag, Calibration_t *high_g) {{
              (void)accel; (void)gyro; (void)mag; (void)high_g;
              return 0;
            }}
            bool mocking_handler_start_mock(void) {{ return false; }}
            bool mocking_handler_cancel_mock(void) {{ return false; }}
            uint32_t dispatch_mock_msg(const uint8_t *message) {{ (void)message; return 0U; }}
            uint32_t mocking_handler_time_from_ring(void) {{ return 0U; }}
            int mocking_handler_read_barometer(BMP581RawData_t *out) {{ (void)out; return 1; }}
            int mocking_handler_read_imu(ICM45686RawData_t *out) {{ (void)out; return 1; }}
            int mocking_handler_read_magnetometer(MMC5983MARawData_t *out) {{
              (void)out;
              return 1;
            }}
            int mocking_handler_read_high_g(ADXL371RawData_t *out) {{ (void)out; return 1; }}

            static void capture_response(TransmitFrame_t *frame) {{
              response_len = frame->payload_len;
              memcpy(response, frame->payload, response_len);
            }}

            int main(void) {{
              commands_set_response_queue(capture_response);
              for (;;) {{
                uint32_t command_len = 0U;
                if (fread(&command_len, sizeof(command_len), 1U, stdin) != 1U) return 0;
                if (command_len == 0U || command_len > 255U) return 1;
                uint8_t command[255];
                if (fread(command, 1U, command_len, stdin) != command_len) return 1;

                response_len = 0U;
                dispatch_command(command);
                fwrite(&response_len, sizeof(response_len), 1U, stdout);
                if (response_len > 0U) fwrite(response, 1U, response_len, stdout);
                fflush(stdout);
              }}
            }}
            """
        ),
        encoding="utf-8",
    )

    include_args = [
        "-I",
        str(STM32_CORE_ROOT / "Inc"),
        "-I",
        str(STM32_CORE_ROOT / "Inc" / "modules"),
        "-I",
        str(STM32_CORE_ROOT / "Inc" / "shared_data"),
        "-I",
        str(STM32_CORE_ROOT / "Inc" / "data_processing"),
        "-I",
        str(STM32_CORE_ROOT / "Inc" / "interfaces"),
        "-I",
        str(REPO_ROOT / "STM32" / "Libraries" / "BMP581"),
        "-I",
        str(REPO_ROOT / "STM32" / "Libraries" / "ICM45686"),
        "-I",
        str(REPO_ROOT / "STM32" / "Libraries" / "MMC5983MA"),
        "-I",
        str(REPO_ROOT / "STM32" / "Libraries" / "ADXL371"),
    ]
    subprocess.run(
        [
            compiler,
            "-std=c99",
            "-O0",
            "-Wall",
            "-Wextra",
            str(harness_c),
            str(STM32_CORE_ROOT / "Src" / "modules" / "commands.c"),
            "-o",
            str(harness_exe),
            *include_args,
        ],
        check=True,
        cwd=build_dir,
    )
    return harness_exe


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
        pytest.skip("TypeScript client dist is missing; build FIRM-Client first.")
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node.js is required for the Web Serial integration test.")

    bridge = RawUsbBridge(compile_stm32_harness(tmp_path))
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
