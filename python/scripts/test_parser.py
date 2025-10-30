from python.firm import FIRM

firm = FIRM(port="/dev/ttyACM0", baudrate=115200)
firm.initialize()
prev_packet_time = None
while True:
    packets = firm.get_data_packets(block=False)
    if prev_packet_time is not None and packets:
        dt = packets[0].timestamp_seconds - prev_packet_time
    if packets:
        prev_packet_time = packets[0].timestamp_seconds
