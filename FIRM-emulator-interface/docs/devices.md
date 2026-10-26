# Emulated devices

What the Teensy must do so the FIRM drivers believe the sensors are real. Everything here comes
from `STM32/Libraries/` and `STM32/Core/Src/main.c`. The register spreadsheet is out of date.

## Sensor bus

All four sensors share STM32 SPI2: mode 0, MSB first, 8-bit words, about 5.25 MHz. Each sensor
has its own chip select and data-ready interrupt line.

| Sensor | STM32 CS | STM32 IRQ | IRQ edge | Teensy CS | Teensy IRQ |
|--------|----------|-----------|----------|-----------|------------|
| ICM45686 (IMU) | PB9 | PB0 | falling | 2 | 23 |
| BMP581 (barometer) | PC2 | PC3 | falling | 3 | 28 |
| MMC5983MA (magnetometer) | PC7 | PB2 | rising | 4 | 29 |
| ADXL371 (high-g) | PA8 | PB1 | falling | 5 | 30 |

STM32 pins are from v1.0; v1.1 pins are not final. Teensy pins are from `FIRM Emulator.pdf`.

Transactions (`spi_utils.c`):

- Read: CS low, 1 address byte, N data bytes, CS high. Burst reads move to the next register
  each byte, except the ICM45686 FIFO_DATA register.
- Write: CS low, address byte, data byte(s), CS high.
- The address byte and the data are sent by two separate HAL calls, so there is a short gap
  after the address byte.

Address byte:

| Sensor | Read | Write |
|--------|------|-------|
| ICM45686, BMP581, MMC5983MA | `reg \| 0x80` | `reg` |
| ADXL371 | `(reg << 1) \| 1` | `reg << 1` |

## Startup

1. `firm_init_hardware()`: flash chip, settings, SD card.
2. Sensor interrupts disabled.
3. Init ICM45686, MMC5983MA, BMP581, ADXL371, in that order. A failure sets an LED code and
   stops FIRM.
4. Load settings, create the log file, write the log header.
5. Sensor interrupts enabled. From here the sensor task reads a sensor on each data-ready edge.

LED codes, read as DEBUG0 (blue, PC1, Teensy 6), DEBUG1 (yellow, PB8, Teensy 9),
DEBUG2 (red, PC5, Teensy 22):

| LEDs | Meaning |
|------|---------|
| 110 | IMU init failed |
| 101 | BMP581 init failed |
| 100 | MMC5983MA init failed |
| 011 | Flash chip failed |
| 010 | High-g init failed |
| 001 | SD card failed |

"FIRM boots with no failure code" is the first check the emulator can make.

## Behavior shared by all sensor models

- A register file: writes are stored, reads return the stored value unless a table below says
  otherwise.
- Soft reset restores the reset values.
- New sample: load the data registers, set the ready flag, assert the IRQ line.
- Latched IRQ lines stay asserted until FIRM reads the status register. As on the real chip, a
  new sample while the line is still asserted makes no new edge.
- Record every write, so a test can check the init sequence.

## ICM45686 (IMU)

Values the driver checks:

| Reg | Name | Emulator returns |
|-----|------|------------------|
| 0x72 | WHO_AM_I | 0xE9 |
| 0x20 | FIFO_CONFIG2 | Reset value 0x20, then whatever was written |
| 0x7F | REG_MISC2 | Bit 1 (soft reset) reads 0 |
| 0x7E | IREG_DATA | Value stored at the indirect address set through 0x7C/0x7D |

Configuration written:

| Reg | Value | Meaning |
|-----|-------|---------|
| 0x7F REG_MISC2 | 0x02 | Soft reset |
| 0x1B ACCEL_CONFIG0 | 0x07 | +-32 g, 400 Hz |
| 0x1C GYRO_CONFIG0 | 0x07 | +-4000 dps, 400 Hz |
| 0x1D FIFO_CONFIG0 | 0x47 | Stream mode, 2 KB |
| 0x21 FIFO_CONFIG3 | 0x0F | FIFO on for accel and gyro |
| 0x16 INT1_CONFIG0 | 0x00, then 0x84 | Interrupt on data ready |
| 0x18 INT1_CONFIG2 | 0x02 | Push-pull, latched, active low |
| 0x10 PWR_MGMT0 | 0x0F | Accel and gyro in low-noise mode |

Indirect registers (16-bit address):

- Write: burst write `[addr high, addr low, data]` starting at 0x7C.
- Read: burst write `[addr high, addr low]` starting at 0x7C, then read 0x7E.
- Written: 0xA267 = 0x02 (big endian), 0xA4A6 = 0x0B, 0xA57B = 0x14, 0xA4AC = 0x01,
  0xA583 = 0x01. The driver reads back 0xA57B and expects 0x14.

Reading a sample:

1. Read 0x19 INT1_STATUS0. Bit 2 is data ready. Reading clears it and releases the IRQ line.
2. Read 1 byte at 0x13: the number of packets in the FIFO. 0 means no data.
3. Read 20 bytes from 0x14 FIFO_DATA per packet. FIRM throws away all but the newest.

The log keeps 15 of the 20 packet bytes, so the emulator rebuilds the packet:

| Packet bytes | Content | Source |
|--------------|---------|--------|
| 0 | Header | Ignored by the driver; use the datasheet value |
| 1-12 | Accel X, Y, Z then gyro X, Y, Z, high byte then low byte | Log bytes 0-11 |
| 13-16 | Temperature and timestamp | Ignored by the driver; fill with 0 |
| 17-19 | Low bits for X, Y, Z (accel high nibble, gyro low nibble) | Log bytes 12-14 |

## BMP581 (barometer)

Values the driver checks (before and after soft reset):

| Reg | Name | Emulator returns |
|-----|------|------------------|
| 0x01 | CHIP_ID | 0x50 |
| 0x02 | REV_ID | 0x32 |
| 0x11 | CHIP_STATUS | 0x02 (SPI mode 0/3; 0x00, 0x01 or bits 0x0C fail) |
| 0x18 | FIFO_SEL | Reset value 0x00, then whatever was written |
| 0x28 | STATUS | 0x02 (bits 2 and 3 must be 0) |
| 0x27 | INT_STATUS | Bit 4 set after soft reset, bit 0 set on new data; cleared on read |

Configuration written:

| Reg | Value | Meaning |
|-----|-------|---------|
| 0x7E CMD | 0xB6 | Soft reset |
| 0x36 OSR_CONFIG | 0x50 | Pressure on, 4x pressure, 1x temperature oversampling |
| 0x14 INT_CONFIG | 0x39 | Interrupt on, active low, latched |
| 0x15 INT_SOURCE | 0x01 | Interrupt on data ready |
| 0x37 ODR_CONFIG | 0x83 | Continuous mode (about 250 Hz) |

Reading a sample: read 0x27; if bit 0 is set, burst read 6 bytes from 0x1D: temperature XLSB,
LSB, MSB, then pressure XLSB, LSB, MSB. This is the same order as the log.

## MMC5983MA (magnetometer)

Values the driver checks:

| Reg | Name | Emulator returns |
|-----|------|------------------|
| 0x2F | PRODUCT_ID1 | 0x30 |
| 0x0A | INTERNAL_CONTROL1 | Bit 7 reads 0 after soft reset (write-only on the real chip) |

Configuration written:

| Reg | Value | Meaning |
|-----|-------|---------|
| 0x0A INTERNAL_CONTROL1 | 0x80 | Soft reset |
| 0x09 INTERNAL_CONTROL0 | 0x08, then 0x10 | SET, then RESET pulse, 10 ms apart |
| 0x09 INTERNAL_CONTROL0 | 0x04 | Interrupt on |
| 0x0A INTERNAL_CONTROL1 | 0x01 | 200 Hz bandwidth |
| 0x0B INTERNAL_CONTROL2 | 0x0E | Continuous mode, 200 Hz |

Reading a sample:

1. Read 0x08 STATUS. Bit 0 is measurement done.
2. FIRM writes 0x01 to 0x08 to clear it.
3. Burst read 7 bytes from 0x00: X high, X mid, Y high, Y mid, Z high, Z mid, then the shared
   low-bits byte. Same order as the log.

The IRQ line is active high: drive it high when a sample is ready and low when FIRM clears bit 0.

## ADXL371 (high-g accelerometer)

Values the driver checks:

| Reg | Name | Emulator returns |
|-----|------|------------------|
| 0x00 | DEVID_AD | 0xAD |
| 0x3A | FIFO_CTL | Reset value 0x00, then whatever was written (write test result is ignored) |

The driver uses read-modify-write on several registers, so stored values must read back.

| Reg | Value | Meaning |
|-----|-------|---------|
| 0x41 RESET | 0x52 | Soft reset |
| 0x3D TIMING | 0x00 | Data rate (comment says 1280 Hz; LOG90.FRM shows about 290 Hz) |
| 0x3B INT1_MAP | Clear, then set 0x81 | Data ready on INT1, active low; bits 1-3 kept |
| 0x3A FIFO_CTL | Clear bits 1-2 | FIFO off |
| 0x3F POWER_CTL | Set bits 2-3 | Filters off |
| 0x3E MEASURE | 0x08 | Low noise, no autosleep |
| 0x3F POWER_CTL | Bits 0-1 = 11, 12 ms later | Measurement mode |

Reset values of INT1_MAP and POWER_CTL are not known yet (assume 0x00 until checked against the
datasheet).

Reading a sample: read 0x04 STATUS; bit 0 is data ready, and reading clears it and releases the
IRQ line. Then burst read 6 bytes from 0x08: X high, X low, Y high, Y low, Z high, Z low (12-bit,
left-justified). Same order as the log.

## Not emulated yet

- W25Q128JV flash: real chip on the board (quad SPI on v1.1, shared with the Teensy through a mux).
- SD card: real card.
- INA219 voltage sensor (I2C): draft driver on `origin/ERM-Voltage-Sensor`, not in main yet.
- GPS (SAM-M10Q), radio (SX1262), pyro, buzzer: v2.0, no drivers yet.
