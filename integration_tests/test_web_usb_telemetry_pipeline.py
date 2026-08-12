"""Regression coverage for the browser client's raw STM32 USB telemetry path."""

from __future__ import annotations

import json
import shutil
import struct
import subprocess
import textwrap
from pathlib import Path

import pytest

# Assertions are the intended pytest API.
# ruff: noqa: S101


REPO_ROOT = Path(__file__).resolve().parents[1]
CLIENT_DIST_INDEX = (
    REPO_ROOT / "FIRM-Client" / "firm_typescript" / "typescript" / "dist" / "index.js"
)


def build_stm32_data_message(
    *, timestamp: float = 42.0, temperature: float = 25.0, pressure: float = 101_325.0
) -> bytes:
    """Build the exact raw telemetry message emitted by ``packetizer_task.c``."""
    # packetizer_task.c sends [ID_DATA_PACKET][DataPacket_t] directly over USB CDC.
    fields = (
        temperature,
        pressure,
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
        6.0,
        7.0,
        8.0,
        9.0,
        10.0,
        11.0,
        12.0,
        13.0,
        14.0,
        1.0,
        0.0,
        0.0,
        0.0,
    )
    return bytes([0x01]) + struct.pack("<d20f", timestamp, *fields)


def parse_message_in_web_client(message: bytes, tmp_path: Path) -> dict[str, object]:
    """Feed raw serial bytes through the built TypeScript and WASM client."""
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node.js is required for the Web/WASM integration test.")

    message_bytes = list(message)
    script = tmp_path / "raw_stm32_telemetry.mjs"
    script.write_text(
        textwrap.dedent(
            f"""
            import {{ readFile }} from 'node:fs/promises';
            import {{ FIRM }} from {json.dumps(CLIENT_DIST_INDEX.as_uri())};

            Object.defineProperty(globalThis, 'navigator', {{
              value: {{ serial: {{}} }},
              configurable: true,
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

            class RawStm32SerialPort {{
              async open() {{
                const message = Uint8Array.from({json.dumps(message_bytes)});
                this.readable = new ReadableStream({{
                  start(controller) {{
                    controller.enqueue(message);
                    controller.close();
                  }},
                }});
                this.writable = new WritableStream();
              }}

              async close() {{}}
            }}

            const firm = await FIRM.connect({{
              baudRate: 115200,
              port: new RawStm32SerialPort(),
            }});
            const packet = await firm.getMostRecentDataPacket();
            console.log(JSON.stringify(packet));
            await firm.close();
            """
        ),
        encoding="utf-8",
    )

    result = subprocess.run(  # noqa: S603 - node and the generated script are controlled here.
        [node, str(script)],
        check=True,
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
    )
    packet: dict[str, object] = json.loads(result.stdout.strip())
    return packet


@pytest.mark.integration
def test_web_client_parses_raw_stm32_usb_telemetry(tmp_path: Path) -> None:
    """Require the Web/WASM client to expose a raw STM32 telemetry message."""
    if not CLIENT_DIST_INDEX.exists():
        pytest.skip("TypeScript client dist is missing; build FIRM-Client first.")

    next_message = build_stm32_data_message(
        timestamp=43.0, temperature=26.0, pressure=100_000.0
    )
    stream = build_stm32_data_message() + next_message[:9]
    packet = parse_message_in_web_client(stream, tmp_path)

    assert packet is not None, "the Web/WASM client discarded the STM32 USB telemetry message"
    assert packet["timestamp_seconds"] == 42.0
    assert packet["temperature_celsius"] == 25.0
    assert packet["pressure_pascals"] == 101_325.0
    assert packet["raw_acceleration_x_gs"] == 1.0
    assert packet["raw_acceleration_y_gs"] == 2.0
    assert packet["raw_acceleration_z_gs"] == 3.0
    assert packet["raw_angular_rate_x_deg_per_s"] == 4.0
    assert packet["raw_angular_rate_y_deg_per_s"] == 5.0
    assert packet["raw_angular_rate_z_deg_per_s"] == 6.0
    assert packet["magnetic_field_x_microteslas"] == 7.0
    assert packet["magnetic_field_y_microteslas"] == 8.0
    assert packet["magnetic_field_z_microteslas"] == 9.0
    assert packet["high_g_accel_x_gs"] == 10.0
    assert packet["high_g_accel_y_gs"] == 11.0
    assert packet["high_g_accel_z_gs"] == 12.0
    assert packet["est_position_z_meters"] == 13.0
    assert packet["est_velocity_z_meters_per_s"] == 14.0
    assert packet["est_quaternion_w"] == 1.0
    assert packet["est_quaternion_x"] == 0.0
    assert packet["est_quaternion_y"] == 0.0
    assert packet["est_quaternion_z"] == 0.0


@pytest.mark.integration
def test_web_client_resynchronizes_after_partial_stm32_telemetry(tmp_path: Path) -> None:
    """Do not decode a payload byte as an ID when attachment occurs mid-packet."""
    if not CLIENT_DIST_INDEX.exists():
        pytest.skip("TypeScript client dist is missing; build FIRM-Client first.")

    # The first byte of this timestamp is 0x01. Simulate opening Web Serial after
    # the true packet ID has already passed, followed by one complete packet.
    first_timestamp = struct.unpack("<d", bytes.fromhex("0100000000004540"))[0]
    partial = build_stm32_data_message(timestamp=first_timestamp)[1:]
    expected = build_stm32_data_message(timestamp=43.0, temperature=26.0, pressure=100_000.0)
    following = build_stm32_data_message(timestamp=44.0, temperature=27.0, pressure=99_500.0)

    packet = parse_message_in_web_client(partial + expected + following[:9], tmp_path)

    assert packet["timestamp_seconds"] == 43.0
    assert packet["temperature_celsius"] == 26.0
    assert packet["pressure_pascals"] == 100_000.0


@pytest.mark.integration
def test_web_client_rejects_false_config_response_inside_telemetry(tmp_path: Path) -> None:
    """Ignore a response ID found in telemetry before the real configuration."""
    if not CLIENT_DIST_INDEX.exists():
        pytest.skip("TypeScript client dist is missing; build FIRM-Client first.")
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node.js is required for the Web/WASM integration test.")

    false_config = bytearray([0x03, 0x01, 0x08])
    false_config.extend(b"A" * (39 - len(false_config)))
    false_config[35:39] = bytes([0, 0, 0, 1])

    real_config = bytearray([0x03])
    real_config.extend(struct.pack("<H", 100))
    real_config.extend(b"FIRM".ljust(32, b"\0"))
    real_config.extend(bytes([1, 0, 0, 0]))
    telemetry = build_stm32_data_message(timestamp=43.0, temperature=26.0, pressure=100_000.0)

    script = tmp_path / "config_response_resync.mjs"
    script.write_text(
        textwrap.dedent(
            f"""
            import {{ readFile }} from 'node:fs/promises';
            import {{ FIRM }} from {json.dumps(CLIENT_DIST_INDEX.as_uri())};

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

            class InterleavedSerialPort {{
              async open() {{
                let controller;
                this.readable = new ReadableStream({{
                  start(value) {{ controller = value; }},
                }});
                this.writable = new WritableStream({{
                  write(command) {{
                    if (command[0] === 0x03) {{
                      controller.enqueue(Uint8Array.from({json.dumps(list(false_config))}));
                      controller.enqueue(Uint8Array.from({json.dumps(list(real_config))}));
                      controller.enqueue(Uint8Array.from({json.dumps(list(telemetry))}));
                    }}
                  }},
                }});
              }}
              async close() {{}}
            }}

            const firm = await FIRM.connect({{
              baudRate: 115200, port: new InterleavedSerialPort(),
            }});
            const config = await firm.getDeviceConfig();
            const packet = await Promise.race([
              firm.getMostRecentDataPacket(),
              new Promise((_, reject) => setTimeout(
                () => reject(new Error('telemetry was not decoded after metadata response')),
                1000,
              )),
            ]);
            console.log(JSON.stringify({{ config, packet }}));
            await firm.close();
            """
        ),
        encoding="utf-8",
    )

    result = subprocess.run(  # noqa: S603 - controlled node executable and script.
        [node, str(script)],
        check=True,
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
    )
    output = json.loads(result.stdout.strip())

    assert output["config"] == {"name": "FIRM", "frequency": 100, "protocol": "USB"}
    assert output["packet"]["timestamp_seconds"] == 43.0
    assert output["packet"]["temperature_celsius"] == 26.0
    assert output["packet"]["pressure_pascals"] == 100_000.0
