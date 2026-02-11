import argparse
import json
import struct
import re
from ripgrepy import Ripgrepy

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

task_mapping = {name[:2]: name for name in names}

def get_function_traces(core_path: str) -> dict:
    """
    Use ripgrep to find all TRACE_END_REGION calls and extract function IDs and names.
    Returns a dict mapping 2-char IDs to full function names.
    """
    function_mapping = {}
    occurrences = {}
    
    try:
        import json
        rg = Ripgrepy("TRACE_END_REGION", core_path).json()
        result = rg.run()
        
        # Parse JSON output
        matches = json.loads(result.as_json)
        
        # Pattern to extract ID and name from TRACE_END_REGION("id", "name")
        pattern = r'TRACE_END_REGION\("(.{2})",\s*"(.+?)"\)'
        
        for match in matches:
            if match.get("type") == "match":
                line_text = match.get("data", {}).get("lines", {}).get("text", "")
                regex_match = re.search(pattern, line_text)
                if regex_match:
                    trace_id = regex_match.group(1)
                    func_name = regex_match.group(2)
                    path_text = match.get("data", {}).get("path", {}).get("text", "unknown")
                    line_number = match.get("data", {}).get("line_number", "?")
                    occurrences.setdefault(trace_id, []).append(
                        (func_name, path_text, line_number)
                    )
                    if trace_id not in function_mapping:
                        function_mapping[trace_id] = func_name
        conflicting_ids = {}
        for trace_id, entries in occurrences.items():
            unique_names = {entry[0] for entry in entries}
            if len(unique_names) > 1:
                conflicting_ids[trace_id] = entries
        if conflicting_ids:
            details = "; ".join(
                f"{trace_id}: {entries}" for trace_id, entries in conflicting_ids.items()
            )
            raise ValueError(f"Conflicting TRACE_END_REGION IDs detected: {details}")
    except Exception as e:
        print(f"Warning: Failed to search for function traces: {e}")
        import traceback
        traceback.print_exc()
    
    return function_mapping

def first_two_letter_to_name(code: bytes, task_mapping: dict, function_mapping: dict) -> tuple:
    """
    Convert 2-letter code to name and determine if it's a function trace.
    Returns (name, is_function_trace).
    """
    # Check if this is a function trace (highest bit of first char set)
    is_function_trace = (code[0] & 0x80) != 0
    
    if is_function_trace:
        # Mask off the high bit to get the original character
        original_code = chr(code[0] & 0x7F) + chr(code[1])
        name = function_mapping.get(original_code, "unknown_function")
    else:
        task_code = chr(code[0]) + chr(code[1])
        name = task_mapping.get(task_code, "unknown")
        if name == "unknown":
            print(f"Unknown trace code: {task_code}")
    
    return name, is_function_trace

def main() -> None:
    parser = argparse.ArgumentParser(description="Convert binary trace to JSON.")
    parser.add_argument("-i", "--input", default="trace.bin", help="Input trace .bin file")
    parser.add_argument("-o", "--output", default="trace.json", help="Output trace .json file")
    parser.add_argument("-c", "--core-path", default="STM32/Core/Src", help="Path to STM32 Core folder for scanning function traces")
    args = parser.parse_args()

    # Get function traces dynamically from source code
    function_mapping = get_function_traces(args.core_path)
    print(f"Found {len(function_mapping)} function traces: {function_mapping}")

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
            name, is_function_trace = first_two_letter_to_name(code, task_mapping, function_mapping)
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

            # if name == "filterDataTask":
            #     continue
            if is_function_trace:
                # Function traces all occur during filterDataTask
                # tid = names.index("filterDataTask") if "filterDataTask" in names else 0
                tid = 20
            else:
                # Task traces
                tid = names.index(name) if name in names else 0
            result.append({ "cat":"function", "name":name, "ph":'X', "pid":0, "tid":tid, "ts":t0, "dur":(t1-t0) })
        json.dump(result, open(args.output, "w"))


if __name__ == "__main__":
    main()