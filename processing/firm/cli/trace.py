"""Convert a FreeRTOS binary trace dump into JSON for Spall or Perfetto."""

import argparse
import json
import struct
from pathlib import Path

# Based on https://emlogic.no/2025/10/poor-mans-freertos-tracing/

TASK_NAMES = [
    "defaultTask",
    "startupTask",
    "systemManagerTask",
    "modeIndicatorTask",
    "bmp581Task",
    "icm45686Task",
    "mmc5983maTask",
    "filterDataTask",
    "packetizerTask",
    "transmitTask",
    "usbReadTask",
    "mockPacketTask",
    "IDLE",
]
TASKS_BY_CODE = {name[:2]: name for name in TASK_NAMES}

# Firmware ring buffer: EVENT_CAPACITY events of (2-char task code, u16 start, u16 end),
# followed by the u32 total event count.
EVENT_CAPACITY = 800
EVENT_FORMAT = "<2sHH"
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

# Timer runs at the 168 MHz core clock divided by 256 and wraps at 16 bits.
CLOCK_FREQUENCY_HZ = 168e6 / 2**8
TIMER_WRAP_SECONDS = 2**16 / CLOCK_FREQUENCY_HZ


def task_name(code: str) -> str:
    if code not in TASKS_BY_CODE:
        print(f"Unknown code: {code}")
    return TASKS_BY_CODE.get(code, "unknown")


def convert(raw: bytes) -> list[dict[str, object]]:
    (event_index,) = struct.unpack_from("<I", raw, EVENT_CAPACITY * EVENT_SIZE)
    write_head = event_index % EVENT_CAPACITY
    event_count = min(event_index, EVENT_CAPACITY)

    events: list[dict[str, object]] = []
    offset_seconds = 0.0
    # Iterate the ring buffer in the order it was written.
    for index in [*range(write_head, event_count), *range(write_head)]:
        code, start_raw, end_raw = struct.unpack_from(EVENT_FORMAT, raw, index * EVENT_SIZE)
        name = task_name(code.decode("utf8"))

        start_us = (start_raw / CLOCK_FREQUENCY_HZ + offset_seconds) * 1e6
        if start_raw > end_raw:
            offset_seconds += TIMER_WRAP_SECONDS
        end_us = (end_raw / CLOCK_FREQUENCY_HZ + offset_seconds) * 1e6

        events.append(
            {
                "cat": "function",
                "name": name,
                "ph": "X",
                "pid": 0,
                "tid": TASK_NAMES.index(name) if name in TASK_NAMES else 0,
                "ts": start_us,
                "dur": end_us - start_us,
            }
        )
    return events


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert binary trace to JSON.")
    parser.add_argument("-i", "--input", default="trace.bin", help="Input trace .bin file")
    parser.add_argument("-o", "--output", default="trace.json", help="Output trace .json file")
    args = parser.parse_args()

    events = convert(Path(args.input).read_bytes())
    with Path(args.output).open("w", encoding="utf-8") as output:
        json.dump(events, output)


if __name__ == "__main__":
    main()
