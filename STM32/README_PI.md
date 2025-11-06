# Raspberry Pi I2C Reader for STM32 Sensor Data

Python tools for reading sensor data from STM32 via I2C interface.

## Overview

The STM32 acts as an I2C slave at address **0x48** and transmits 58-byte sensor packets containing:
- Timestamp (64-bit)
- Pressure data (32-bit raw + 8-bit fractional)
- 3-axis accelerometer (3 floats)
- 3-axis gyroscope (3 floats)
- 3-axis magnetometer (3 floats)
- Temperature (double)
- Altitude (float)
- CRC-16 checksum

## Files

- **`pi_i2c_reader.py`** - Main I2C reader library with packet parsing
- **`pi_test_i2c.py`** - Test suite to verify I2C communication
- **`pi_data_logger.py`** - Data logger that saves packets to CSV files
- **`README_PI.md`** - This file

## Setup

### 1. Install Dependencies

On your Raspberry Pi Zero, install required packages:

```bash
# Update package list
sudo apt-get update

# Install Python 3 and pip
sudo apt-get install -y python3 python3-pip

# Install smbus2 library
pip3 install smbus2
```

### 2. Enable I2C

Enable the I2C interface on your Raspberry Pi:

```bash
# Enable I2C
sudo raspi-config
# Navigate to: Interface Options -> I2C -> Enable

# Reboot
sudo reboot
```

### 3. Verify I2C Connection

After connecting the STM32 (SDA to GPIO2, SCL to GPIO3, GND to GND), verify:

```bash
# Install i2c-tools
sudo apt-get install -y i2c-tools

# Scan for I2C devices (should see 0x48)
i2cdetect -y 1
```

Expected output:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- 48 -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --
```

## Usage

### Test Communication

Run the test suite to verify everything is working:

```bash
python3 pi_test_i2c.py
```

This will run 5 tests:
1. Device detection
2. Single byte read
3. Multi-byte read (header)
4. Full packet read (58 bytes)
5. Repeated reads

### Read Single Packet

```python
from pi_i2c_reader import STM32I2CReader

reader = STM32I2CReader(bus_number=1)
packet = reader.read_packet()

if packet:
    print(f"Temperature: {packet.temperature:.2f}°C")
    print(f"Altitude: {packet.altitude:.2f}m")
    print(f"Accel: ({packet.accel_x:.3f}, {packet.accel_y:.3f}, {packet.accel_z:.3f}) m/s²")
```

### Continuous Reading

```python
from pi_i2c_reader import STM32I2CReader

reader = STM32I2CReader(bus_number=1)

def process_packet(packet):
    print(f"Temp: {packet.temperature:.1f}°C, Alt: {packet.altitude:.1f}m")

reader.read_continuous(interval=0.1, callback=process_packet)
```

### Data Logging

Log data to CSV files:

```bash
python3 pi_data_logger.py
```

CSV files are created in `sensor_logs/` directory with format:
```
sensor_data_YYYYMMDD_HHMMSS.csv
```

CSV columns:
- `system_time` - Pi system timestamp (ISO format)
- `stm32_timestamp` - STM32 clock cycle counter
- `pressure_pa` - Pressure in Pascals
- `accel_x/y/z_ms2` - Acceleration (m/s²)
- `gyro_x/y/z_rads` - Angular velocity (rad/s)
- `mag_x/y/z_ut` - Magnetic field (µT)
- `temperature_c` - Temperature (°C)
- `altitude_m` - Altitude (m)
- `crc_valid` - CRC checksum valid (True/False)

## Packet Structure

The 58-byte packet structure:

| Offset | Size | Type    | Field           | Description                    |
|--------|------|---------|-----------------|--------------------------------|
| 0      | 2    | uint16  | header          | Magic: 0xA55A                  |
| 2      | 2    | uint16  | length          | Payload length: 56             |
| 4      | 8    | uint64  | timestamp       | STM32 clock cycles             |
| 12     | 4    | uint32  | pressure_raw    | Pressure integer part          |
| 16     | 1    | uint8   | pressure_frac   | Pressure fractional (*1e-9)    |
| 17     | 4    | float   | accel_x         | Accelerometer X (m/s²)         |
| 21     | 4    | float   | accel_y         | Accelerometer Y (m/s²)         |
| 25     | 4    | float   | accel_z         | Accelerometer Z (m/s²)         |
| 29     | 4    | float   | gyro_x          | Gyroscope X (rad/s)            |
| 33     | 4    | float   | gyro_y          | Gyroscope Y (rad/s)            |
| 37     | 4    | float   | gyro_z          | Gyroscope Z (rad/s)            |
| 41     | 4    | float   | mag_x           | Magnetometer X (µT)            |
| 45     | 4    | float   | mag_y           | Magnetometer Y (µT)            |
| 49     | 4    | float   | mag_z           | Magnetometer Z (µT)            |
| 53     | 8    | double  | temperature     | Temperature (°C)               |
| 61     | 4    | float   | altitude        | Altitude (m)                   |
| 65     | 4    | uint32  | reserved        | Reserved                       |
| 69     | 2    | uint16  | crc             | CRC-16-CCITT (KERMIT)          |

All values are little-endian.

## Troubleshooting

### Device not detected

```bash
# Check I2C is enabled
ls /dev/i2c-*

# Check wiring
i2cdetect -y 1

# Check permissions
sudo usermod -a -G i2c pi
```

### Timeout errors

- Verify STM32 is powered and running
- Check pull-up resistors on SDA/SCL (should be 4.7kΩ to 10kΩ)
- Reduce I2C clock speed if wires are long
- Try reading smaller chunks with `read_packet_chunked()`

### CRC errors

- Usually indicates communication errors
- Check wiring and pull-ups
- Ensure STM32 firmware is flashed correctly
- Try slower I2C clock speed

### No new data

- STM32 buffer updates only when sensors have new data
- Trigger sensor interrupts by moving/tilting the board
- Check that `firmSettings.serial_transfer_enabled` is true on STM32

## Hardware Connections

```
Raspberry Pi Zero          STM32F405
GPIO2 (Pin 3)  ──────────  PB11 (I2C2_SDA)
GPIO3 (Pin 5)  ──────────  PB10 (I2C2_SCL)
GND (Pin 6)    ──────────  GND

Pull-ups (optional if already on board):
  SDA: 4.7kΩ to 3.3V
  SCL: 4.7kΩ to 3.3V
```

## API Reference

### STM32I2CReader

```python
reader = STM32I2CReader(bus_number=1)
```

Methods:
- `read_packet()` - Read full packet at once
- `read_packet_chunked(chunk_size=16)` - Read in smaller chunks
- `read_continuous(interval=0.1, callback=None)` - Continuous reading

### SensorPacket

Attributes:
- `header`, `length`, `timestamp`
- `pressure_raw`, `pressure_frac`
- `accel_x/y/z`, `gyro_x/y/z`, `mag_x/y/z`
- `temperature`, `altitude`
- `crc`, `crc_valid`

### SensorDataLogger

```python
with SensorDataLogger(output_dir="sensor_logs") as logger:
    logger.log_packet(packet)
```

## Example: Real-time Display

```python
#!/usr/bin/env python3
from pi_i2c_reader import STM32I2CReader
import time

reader = STM32I2CReader()

while True:
    packet = reader.read_packet()
    if packet and packet.crc_valid:
        print(f"\rTemp: {packet.temperature:6.2f}°C | "
              f"Alt: {packet.altitude:7.2f}m | "
              f"Accel: {packet.accel_x:6.2f} {packet.accel_y:6.2f} {packet.accel_z:6.2f} m/s²",
              end='', flush=True)
    time.sleep(0.1)
```

## License

Part of the NCSU High-Powered Rocketry Club FIRM project.
