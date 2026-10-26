# FIRM Emulator

A test board that runs the unmodified FIRM launch firmware on a real STM32, while a Teensy 4.1
pretends to be its sensors and replays recorded flights. The flight firmware contains no mock code.

```
Laptop (Python) --USB--> Teensy 4.1 (C++) --SPI + interrupt lines--> STM32 running FIRM
```

## Decisions

- Target FIRM v1.1 first, then v2.0 (v1.1 plus radio, GPS, pyro, buzzer).
- Sensor behavior comes from the drivers in `STM32/Libraries/`, not the register spreadsheet.
- The flight log is written to the SD card. The SD card and flash are real chips on the board.
- First test: the log FIRM writes during a replay matches the log that was replayed.

## Layout

```
FIRM-emulator-interface/
  README.md            this file
  docs/
    devices.md         what each emulated sensor must do
    log-format.md      .frm file layout and the matching-log test
    protocol.md        laptop <-> Teensy messages          (iteration 3)
  host/                Python package firm_emu             (iterations 1-3)
    pyproject.toml
    firm_emu/
      cli.py           firm-emu inspect | compare | run
      log.py           read, write, and compare .frm logs
      replay.py        replay at the original timing
      protocol.py      message framing                     (iteration 3)
    tests/
  teensy/              PlatformIO project                  (iteration 4+)
    platformio.ini
    src/
      main.cpp
      host_link.cpp    USB messages and sample buffer
      spi_slave.cpp    sensor bus
      sensors.cpp      the four sensor models
```

Folders are created by the iteration that first needs them.

## Host tools

Run from `host/` (needs [uv](https://docs.astral.sh/uv/)):

```
uv sync
uv run firm-emu inspect LOG90.FRM
uv run firm-emu compare replayed.frm firm_output.frm
uv run firm-emu run LOG90.FRM --out replay.frm --seconds 60
uv run firm-emu compare LOG90.FRM replay.frm --seconds 60
uv run pytest
```

- `compare` prints a table per sensor and ends with PASS or FAIL (exit code 0 or 1). The rules
  are in `docs/log-format.md`.
- `run` replays a log at its original timing. For now it writes to a file, stamping each record
  with the time it was actually sent, so `compare` shows how well the timing held. `--speed`
  replays faster, `--seconds` replays only the start of the log (use the same `--seconds` with
  `compare`).

## Iterations

| # | Goal | Done when | Status |
|---|------|-----------|--------|
| 0 | Specs from the drivers | `docs/devices.md` and `docs/log-format.md` written | Done |
| 1 | Read and compare logs | `firm-emu inspect` and `firm-emu compare` work on LOG90.FRM | Done |
| 2 | Replay timing | Replay to a file keeps the logged timing | Done |
| 3 | Laptop-Teensy protocol | `firm-emu run --fake` works against a Python stand-in | Next |
| 4 | Teensy SPI slave test | Teensy answers the BMP581 chip ID read from a real STM32 | |
| 5 | First sensor | FIRM reads replayed BMP581 data | |
| 6 | All four sensors | FIRM boots with no failure LED | |
| 7 | Matching-log test | Replayed log and FIRM's log match on real hardware | |
| 8 | PCB | Board brought up | |
| 9 | Automation | Tests run from a script, then CI | |
| 10 | v2.0 devices | Radio, GPS, voltage sensor, pyro, buzzer | |

Iterations 1-3 need no hardware. Iteration 4 decides whether the Teensy-only design works, so do
it early.

## Open questions

- STM32 pins for v1.1 (in progress).
- Teensy pin 10 is the only chip select the SPI slave can watch. The four sensor chip selects need
  a gate (for example a 74LVC21) to combine them.
- Teensy pins 42-45 are planned for the flash. Does the Teensy also need to read the SD card?
- How to observe the pyro outputs safely, and whether to route STM32 reset and SWD.
- Datasheet values still needed: ADXL371 reset values for INT1_MAP and POWER_CTL, and the
  ICM45686 FIFO header byte.
