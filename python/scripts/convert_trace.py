import argparse
import json
import struct

# Based on https://emlogic.no/2025/10/poor-mans-freertos-tracing/

names = [
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
    "IDLE"
]

mapping = {name[:2]: name for name in names}

def first_two_letter_to_name(code: str) -> str:
    if code not in mapping:
        print(f"Unknown code: {code}")
    return mapping.get(code, "unknown")

def main() -> None:
    parser = argparse.ArgumentParser(description="Convert binary trace to JSON.")
    parser.add_argument("-i", "--input", default="trace.bin", help="Input trace .bin file")
    parser.add_argument("-o", "--output", default="trace.json", help="Output trace .json file")
    args = parser.parse_args()

    result = []
    raw = open(args.input, "rb").read()

    N = 800
    event_index, = struct.unpack_from("<I", raw, N*6) # (each event is 24 bytes)
    event_write_head = event_index % N
    event_count = min(event_index, N)

    clock_frequency = 168e6 / (2 ** 8) # 168 Mhz clock

    last_t0 = 0
    offset = 0

    temp = 0

    # Iterate ring buffer in the same order as it was written to.
    for r in [range(event_write_head, event_count), range(0, event_write_head)]:
        for index in r:
            # name, t0_raw, t1_raw = struct.unpack_from("<16sII", raw, index*24)
            code, t0_raw, t1_raw = struct.unpack_from("<2sHH", raw, index*6)
            name = first_two_letter_to_name(code.decode("utf8"))
            # Overflow detection
            # if t0_raw < last_t0:
            #     offset += (2 ** 16) / clock_frequency
            #     print("jumping")
            # last_t0 = t0_raw

            # print(code, t0_raw, t1_raw)

            t0 = (t0_raw / clock_frequency + offset) * 1e6  # convert to microseconds
            if t0_raw > t1_raw:
                offset += (2 ** 16) / clock_frequency
                # result.append({ "cat":"function", "name":"Overflow", "ph":'X', "pid":0, "tid":99, "ts":t0, "dur":(t1-t0) })
            t1 = (t1_raw / clock_frequency + offset) * 1e6  # convert to microseconds

            # t0 = temp
            # t1 = temp + 100
            # temp += 200

            # temp += 1
            # if temp > 10:
            #     break
                
            # print(t0, t1, name)

            tid = names.index(name) if name in names else 0
            result.append({ "cat":"function", "name":name, "ph":'X', "pid":0, "tid":tid, "ts":t0, "dur":(t1-t0) })
        json.dump(result, open(args.output, "w"))


if __name__ == "__main__":
    main()