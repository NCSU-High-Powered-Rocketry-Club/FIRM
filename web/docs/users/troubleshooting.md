# Using FIRM - Troubleshooting

This page collects **common problems** that end users might encounter and suggestions about how to resolve them.

---

## Device Won't Show Up in the Port List

Possible causes:

- USB cable is **power-only** (try a known good data cable).
- USB-C is flipped the wrong way. Unplug it and replug it rotated 180 degrees. _We don't know why this happens, but we're working on a fix._
- Board not powered, physically damaged, or MicroSD card not inserted properly.

Checklist:

1. Try a different cable and USB port.
2. Make sure you see only a green LED on the board when you first plug it in.
3. Check your Device Manager for a new COM port that appears when you plug in the board.

---

## LED Status Codes

The board uses 3 LEDs (Blue, Yellow, Red) to indicate operational status.

### Initialization Status

These LEDs will always stay lit up to indicate the initialization status. When FIRM is booting up, you should see all 3 LEDs lit for 0.5 seconds.

| Status          | Blue | Yellow | Red |
| :-------------- | :--: | :----: | :-: |
| Bootup          |  🔵  |   🟡   | 🔴  |
| IMU Fail        |  🔵  |   🟡   | ⚫  |
| BMP581 Fail     |  🔵  |   ⚫   | 🔴  |
| MMC5983MA Fail  |  🔵  |   ⚫   | ⚫  |
| Flash Chip Fail |  ⚫  |   🟡   | 🔴  |
| SD Card Fail    |  ⚫  |   🟡   | ⚫  |
| All Sensors OK  |  ⚫  |   ⚫   | ⚫  |

### Interrupt Failures

If a sensor's interrupt is not firing, the LEDs will blink 5 times.

| Status                        | Blue | Yellow | Red |
| :---------------------------- | :--: | :----: | :-: |
| Failed IMU Interrupt          |  🔵  |   ⚫   | ⚫  |
| Failed BMP Interrupt          |  ⚫  |   🟡   | ⚫  |
| Failed Magnetometer Interrupt |  ⚫  |   ⚫   | 🔴  |

## Web App Can't Connect via Serial

- Make sure you're using a **supported browser** (Chrome/Edge work well, don't use Firefox).
- Ensure no other process is holding the serial port (e.g., another terminal, or web app tab).
- If in doubt, unplug and replug the device and restart the browser.

---

## No Data in Logs

TODO: idk about this one, need to test

---

## Data Looks Wrong (e.g., Saturated, Noisy, or Offset)

TODO: merge this with No Data in Logs

---

## When to Ask for Help

If you've gone through the relevant sections, and you're still stuck, contact someone from the FIRM team for help.

---
