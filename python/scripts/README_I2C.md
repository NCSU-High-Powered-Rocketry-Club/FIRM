# FIRM I2C Communication with Raspberry Pi Zero

This document describes how to read sensor data from the STM32 FIRM board via I2C using a Raspberry Pi Zero.

## Hardware Setup

### Wiring
Connect the following pins between the Pi Zero and the STM32:

| Raspberry Pi Zero | STM32 (STM32F405) | Description |
|-------------------|-------------------|-------------|
| GPIO2 (Pin 3)     | PB11 (I2C2_SDA)   | I2C Data    |
| GPIO3 (Pin 5)     | PB10 (I2C2_SCL)   | I2C Clock   |
| GND               | GND               | Ground      |

**Important notes:**
- Both devices operate at **3.3V** — no level shifters needed
- The Pi has built-in pull-up resistors on the I2C pins (typically 1.8kΩ to 3.3V)
- Ensure the STM32 board doesn't have conflicting pull-ups to a different voltage
- **Do not** connect the Pi to the STM32's I2C1 bus (PB6/PB7) — that bus is used for onboard sensors

### STM32 Configuration
The STM32 firmware is configured with:
- I2C2 slave address: **0x42** (7-bit)
- In the firmware, `hi2c2.Init.OwnAddress1 = (0x42 << 1)` = 0x84 (HAL expects shifted value)
- I2C2 operates in slave mode with interrupt-driven callbacks
- The STM32 serves the latest serialized sensor packet (62 bytes) when read by the Pi

## Raspberry Pi Setup

### 1. Enable I2C on the Pi
```bash
sudo raspi-config
```
Navigate to:
- **Interfacing Options** → **I2C** → **Enable**

Reboot:
```bash
sudo reboot
```

Verify I2C is enabled:
```bash
ls /dev/i2c*
# Should show /dev/i2c-1
```

### 2. Install Required Python Packages
On the Pi:
```bash
# Install system I2C tools
sudo apt-get update
sudo apt-get install -y i2c-tools python3-smbus

# Install smbus2 (pure Python I2C library)
pip3 install smbus2
```

### 3. Test I2C Detection
With the STM32 powered and connected:
```bash
i2cdetect -y 1
```

You should see address **0x42** appear in the grid. Example:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
...
40: -- -- 42 -- -- -- -- -- -- -- -- -- -- -- -- --
...
```

**Note:** `i2cdetect` may show the address intermittently due to how it probes. A direct read (see below) is more reliable.

## Reading Sensor Data

### Quick Test
Use the provided test script to read raw bytes:
```bash
cd /home/pi/FIRM/python/scripts
python3 pi_i2c_reader.py
```

This will continuously read and decode sensor packets, displaying:
- Temperature (°C)
- Pressure (Pa)
- Acceleration (g)
- Angular rate (rad/s)
- Magnetic field (µT)
- Timestamp (seconds)

### Example Output
```
FIRM I2C Sensor Reader
Reading from STM32 at I2C address 0x42 on bus 1
--------------------------------------------------------------------------------
Raw (62 bytes): 5aa53800000000000000000000000000004097bd0000d3bc00e880bf98882eba57a08b3a60b798bb78bac7c44806c5c400f6d1c1000000003359
  Header: 0xA55A  Length: 56  CRC: OK
  Temperature: 0.00 °C
  Pressure:    0.00 Pa
  Accel:       ( -0.092,   0.000,  -1.068) g
  Gyro:        (  0.000,   0.000,   0.000) rad/s
  Mag:         (  0.001,  -0.091,  -0.096) µT
  Timestamp:   3250479.000000 s
```

## Packet Format

The STM32 sends a **62-byte** serialized packet:

| Offset | Field             | Type     | Size | Description                          |
|--------|-------------------|----------|------|--------------------------------------|
| 0      | header            | uint16   | 2    | Sync header (0xA55A)                 |
| 2      | length            | uint16   | 2    | Payload size (52 bytes)              |
| 4-7    | (padding)         | -        | 4    | Struct alignment padding             |
| 8      | temperature       | float    | 4    | Temperature in °C                    |
| 12     | pressure          | float    | 4    | Pressure in Pascals                  |
| 16     | accel_x,y,z       | float[3] | 12   | Acceleration in g                    |
| 28     | angular_rate_x,y,z| float[3] | 12   | Angular rate in rad/s                |
| 40     | mag_x,y,z         | float[3] | 12   | Magnetic field in µT                 |
| 52     | timestamp_sec     | double   | 8    | Clock cycles converted to seconds    |
| 60     | crc               | uint16   | 2    | CRC-16-CCITT (Kermit)                |

All multi-byte values are **little-endian**.

## Troubleshooting

### Address not detected (`i2cdetect` shows all `--`)
1. **Check wiring:**
   - SDA, SCL, and GND properly connected
   - No shorts or loose connections
2. **Verify pull-ups:**
   - Pi I2C pins have internal pull-ups; ensure no conflicting external pull-ups
3. **Check STM32 firmware:**
   - I2C2 peripheral is initialized (`MX_I2C2_Init()` called)
   - GPIO pins PB10/PB11 configured as I2C2 AF (alternate function) with open-drain
   - I2C2 interrupts enabled in NVIC (I2C2_EV_IRQn, I2C2_ER_IRQn)
   - `HAL_I2C_EnableListen_IT(&hi2c2)` succeeds without error
4. **Try different address format:**
   - If still no detection, try changing `hi2c2.Init.OwnAddress1` in firmware:
     ```c
     hi2c2.Init.OwnAddress1 = 0x42;  // instead of (0x42 << 1)
     ```
5. **Use a logic analyzer:**
   - Capture SDA/SCL during `i2cdetect` or Python read to see actual bus transactions

### CRC failures
- Indicates data corruption or struct alignment mismatch
- Verify the C struct packing matches Python unpacking (4-byte padding after `length`)
- Check for EMI/noise on I2C lines (use shorter wires, add decoupling caps)

### Stale data (same reading every time)
- The STM32 only updates `i2c2_tx_buf` when:
  - `firmSettings.serial_transfer_enabled` is true, AND
  - New sensor data is collected
- To always populate the I2C buffer, move the `memcpy` outside the `if (firmSettings.serial_transfer_enabled ...)` block in `main.c`

### I2C bus hangs
- Check for clock stretching issues
- Verify both devices are not holding SDA/SCL low
- Power-cycle both devices to reset I2C state machines
- Add `HAL_I2C_ErrorCallback` handling (already included in updated firmware)

## Advanced Usage

### Reading on demand
To trigger a fresh sensor sample from the Pi:
1. Implement a command protocol where the Pi writes a request byte to the STM32
2. In `HAL_I2C_SlaveRxCpltCallback`, check for the command and prepare a new packet
3. Pi then reads the response

### Continuous streaming
For higher data rates, consider:
- Increasing I2C clock speed (currently 100 kHz, can go to 400 kHz)
- Using DMA for I2C transfers on the STM32
- Buffering multiple packets on the STM32 side

## Reference Files
- **STM32 firmware:** `STM32/Core/Src/main.c` (I2C2 slave implementation)
- **Python reader:** `python/scripts/pi_i2c_reader.py` (packet parser and display)
- **Packet structure:** `STM32/Core/Inc/usb_serializer.h` (SerializedPacket_t definition)

## Support
For issues or questions, check:
- [FIRM GitHub Repository](https://github.com/NCSU-High-Powered-Rocketry-Club/FIRM)
- STM32 HAL I2C documentation
- smbus2 Python library documentation
