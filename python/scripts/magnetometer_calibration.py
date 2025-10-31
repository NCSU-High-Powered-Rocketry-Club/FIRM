from python.firm import FIRM, calibrate_magnetometer

firm = FIRM(port="/dev/ttyACM0", baudrate=115200)
firm.initialize()

print(calibrate_magnetometer(firm, 60))
