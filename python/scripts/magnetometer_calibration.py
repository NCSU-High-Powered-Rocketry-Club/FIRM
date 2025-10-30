from python.firm import calibrate_magnetometer
from python.firm import FIRM

firm = FIRM(port="/dev/ttyACM0", baudrate=115200)
firm.initialize()

print(calibrate_magnetometer(firm, 60))
