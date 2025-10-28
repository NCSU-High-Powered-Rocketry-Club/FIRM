import time

from firm import FIRM

firm = FIRM(port="/dev/ttyACM0", baudrate=115200, one_shot_mode=True)
firm.initialize()
prev_packet_time = None
while True:
    # packets = firm.get_data_packets(block=False)
    packet = firm.get_most_recent_data_packet()
    # print(packets)
    # time.sleep(1)
    # print()
    # if prev_packet_time is not None and packets:
    #     dt = packets[0].timestamp_seconds - prev_packet_time
    #     print(f"dt: {dt * 1000:.2f} ms", "len packets:", len(packets))
    # if packets:
        # print(packets[0])
        # prev_packet_time = packets[0].timestamp_seconds
    if packet:
        print(packet)

