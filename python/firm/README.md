## FIRM Python Library


This directory contains the Python library for FIRM, which provides tools and utilities for working with the FIRM firmware and hardware components.


## Installation

You can install the FIRM Python library using pip:

```bash
pip install firm-hprc
```

## Usage

After installing the library, you can import it in your Python scripts:

```python
from firm import FIRM

firm = FIRM("/path/to/device")  # e.g. "/dev/ttyUSB0" or "COM3"
firm.initialize()
data = f.get_data_packets()
print(data[0].accel_x_meters_per_s2)
```

The returned data packet is a `FIRMPacket`, which contains various sensor readings and status information. See the source code [here](https://github.com/NCSU-High-Powered-Rocketry-Club/FIRM/blob/main/python/firm/packets.py) on what fields are available.

