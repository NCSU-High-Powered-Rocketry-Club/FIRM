from firm.parser import PacketParser

pp = PacketParser(port="COM3", baudrate=115200)
pp.initialize()
pp.get_data_packets(block=True)
