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
2. Make sure the green power LED is on, and that after a moment the yellow LED is blinking (see below).
3. Check your Device Manager for a new COM port that appears when you plug in the board.

---

## LED Status Codes

Besides the green power LED, the board uses 3 status LEDs (Blue, Yellow, Red).

### Normal Operation

| Status                      | Blue | Yellow | Red |
| :-------------------------- | :--: | :----: | :-: |
| Booting (blinking)          |  🔵  |   ⚫   | ⚫  |
| Running normally (blinking) |  ⚫  |   🟡   | ⚫  |

### Initialization Failures

If a component fails to start, FIRM stops and these LEDs stay lit (not blinking).

| Status                    | Blue | Yellow | Red |
| :------------------------ | :--: | :----: | :-: |
| IMU Fail                  |  🔵  |   🟡   | ⚫  |
| BMP581 Fail               |  🔵  |   ⚫   | 🔴  |
| MMC5983MA Fail            |  🔵  |   ⚫   | ⚫  |
| Flash Chip Fail           |  ⚫  |   🟡   | 🔴  |
| High-g Accelerometer Fail |  ⚫  |   🟡   | ⚫  |
| SD Card Fail              |  ⚫  |   ⚫   | 🔴  |

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
