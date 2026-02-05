from firm import FIRM

firm = FIRM(port="COM8", baudrate=921600)
firm.initialize()
firm.zero_out_pressure_altitude()
prev_packet_time = None
while True:
    packets = firm.get_data_packets(block=False)
    if prev_packet_time is not None and packets:
        dt = packets[0].timestamp_seconds - prev_packet_time
        #print(f"dt: {dt * 1000:.2f} ms", "len packets:", len(packets))
    if packets:
        print(packets[0])
        prev_packet_time = packets[0].timestamp_seconds
