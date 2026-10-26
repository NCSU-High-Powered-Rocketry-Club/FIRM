# FIRM log format (.frm v1.4)

How FIRM writes flight logs, and what "the logs match" means for the emulator. Sources:
`STM32/Core/Src/modules/logger.c`, `STM32/Core/Src/interfaces/logger_storage.c`,
`STM32/Core/Src/target/firm_v1_0.c`, `STM32/Core/Inc/shared_data/system_settings.h`.

## File

- Saved on the SD card as `logN.frm`, using the next unused N.
- Created at 200,000,000 bytes. Only the start holds data; the rest is whatever was already on
  the card.
- Written in 8192-byte blocks. A record never crosses a block boundary, and the unused end of each
  block is filled with zeros.

## Header (260 bytes, little endian, no padding)

| Offset | Size | Field |
|--------|------|-------|
| 0 | 14 | `FIRM LOG v1.4\n` |
| 14 | 8 | Device UID (from the flash chip) |
| 22 | 32 | Device name |
| 54 | 4 | Output enabled: USB, UART, I2C, SPI (1 byte each) |
| 58 | 8 | Firmware version |
| 66 | 2 | Output frequency (Hz) |
| 68 | 192 | Calibration for accel, gyro, mag, high-g: each 3 offsets + 3x3 matrix, float32 |

## Records

Records follow the header back to back:

| Size | Field |
|------|-------|
| 1 | Sensor ID: `B` barometer, `I` IMU, `M` magnetometer, `A` high-g |
| 4 | Timestamp: CPU cycle counter at 168 MHz, uint32 |
| N | Raw register bytes, exactly as read from the sensor |

| ID | Sensor | N |
|----|--------|---|
| `B` | BMP581 | 6 |
| `I` | ICM45686 | 15 |
| `M` | MMC5983MA | 7 |
| `A` | ADXL371 | 6 |

Byte order inside each record is described in `devices.md`.

## Timestamps

- The cycle counter wraps every 25.57 s. To get elapsed time, add the difference between
  consecutive records modulo 2^32.
- The timestamp is taken just before FIRM reads the sensor, so it marks when FIRM handled the
  interrupt, not when the sensor took the sample.

## End of data

Data ends at the first block that holds an invalid record or non-zero padding. Everything after
that is old card contents and must be ignored. The existing Python decoder
(`python/firm/flight_data/formats.py`) stops at the first byte that is not a sensor ID, which
gives the same end point for LOG90.FRM.

## Dropped samples

If the SD card is busy when the logger needs a new block, that sample is not logged
(`logger_storage_malloc_capacity` returns NULL). A gap in the log does not always mean the
sensor missed a sample.

## Example: LOG90.FRM

- Firmware v2.2.0, device name "FIRM Device", identity calibration (an uncalibrated recording).
- Data ends at byte 9,617,408 of 200,000,000.
- 671,558 records over 576 s (9.6 min), about 17 KB/s.
- One 50 ms gap, at 571 s. Otherwise 99% of intervals are within 0.2 ms of the median.

| Sensor | Records | Median interval | Rate |
|--------|---------|-----------------|------|
| Barometer | 144,347 | 3.99 ms | 250 Hz |
| IMU | 232,097 | 2.48 ms | 403 Hz |
| Magnetometer | 127,513 | 4.52 ms | 221 Hz |
| High-g | 167,601 | 3.44 ms | 291 Hz |

## Replay

For each record, at its elapsed time, load its bytes into that sensor model and raise data-ready.
The laptop streams records ahead of time and the Teensy keeps its own clock, so USB delays do not
affect timing. The whole log does not fit in Teensy RAM, but a 256 KB buffer holds about 15 s.

`firm-emu run` (`host/firm_emu/replay.py`) runs the same schedule on the laptop: wait until each
record is due, then send it. It sleeps for most of each wait and spins for the last 3 ms, and
turns off Python's garbage collector while it runs.

A full real-time replay of LOG90.FRM on a Windows laptop sent 99.94% of records within 1 ms.
The other 424 were up to 15 ms late, when Windows paused the process. This is why the Teensy, not
the laptop, keeps time in the real system.

## Matching-log test

After a replay, `firm-emu compare` checks FIRM's new log against the replayed log:

- The header is ignored. The UID and calibration come from the emulator board's flash.
- Times are measured from the first record of each log.
- For each sensor, records are paired in time order by identical bytes, then their times are
  checked against the tolerance (default 1 ms).
- The clock offset slowly follows the paired records, so drift between boards passes (a 20 ppm
  drift adds up to about 12 ms over LOG90.FRM) but one late record does not shift the others.

Each expected record ends up as one of:

| Result | Meaning |
|--------|---------|
| matched | Same bytes, on time |
| late | Same bytes, but further off than the tolerance |
| missing | Not in FIRM's log |
| wrong | At the right time, but with different bytes |

Records in FIRM's log that were never replayed are counted as extra.

A sensor fails if it has any late, extra, or wrong record, or more than 1% missing records. A few
missing records are allowed because FIRM drops samples when the SD card is busy.
