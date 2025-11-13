from firm import FIRM, calibrate_magnetometer

firm = FIRM(port="/dev/ttyACM0", baudrate=115200)
firm.initialize()

calibrate_magnetometer(firm)
