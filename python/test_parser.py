import time
from firm.parser import PacketParser

pp = PacketParser(port="/dev/ttyACM0", baudrate=115200)
pp.initialize()
prev_packet_time = None
while True:
    packets = pp.get_data_packets(block=True)
    # print(packets)
    time.sleep(0.3)
    # print()
    if prev_packet_time is not None and packets:
        dt = packets[0].timestamp_seconds - prev_packet_time
        print(f"dt: {dt * 1000:.2f} ms", "len packets:", len(packets))
    if packets:
        prev_packet_time = packets[0].timestamp_seconds
