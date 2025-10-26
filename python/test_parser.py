from firm.parser import PacketParser

pp = PacketParser(port="/dev/ttyACM0", baudrate=115200)
pp.initialize()
while True:
    packets = pp.get_data_packets(block=True)
    print(packets)
