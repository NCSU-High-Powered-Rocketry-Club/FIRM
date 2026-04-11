import argparse
import firm_client

TIMEOUT = 0.1
RESPONSE_TIMEOUT = 5.0


parser = argparse.ArgumentParser(description="Run FIRM device commands")
parser.add_argument("port", help="Serial port name (e.g., COM8 or /dev/ttyACM0)")
parser.add_argument(
    "-b",
    "--baud-rate",
    type=int,
    default=2_000_000,
    help="Baud rate for serial communication (default: 2000000)",
)
args = parser.parse_args()

client = firm_client.FIRMClient(args.port, args.baud_rate, TIMEOUT)
client.start()

device_info = client.get_device_info(timeout_seconds=RESPONSE_TIMEOUT)
if device_info:
    print(f"Version: {device_info.firmware_version}, Id: {device_info.id}, ")

print(
    client.set_device_config(
        "name",
        102,
        firm_client.DeviceProtocol.UART,
        timeout_seconds=RESPONSE_TIMEOUT,
    )
)

device_config = client.get_device_config(timeout_seconds=RESPONSE_TIMEOUT)
if device_config:
    print(
        f"Name: {device_config.name}, Frequency: {device_config.frequency}, Protocol: {device_config.protocol}"
    )
