# Using FIRM - Troubleshooting

This page collects **common problems** that end users might encounter and suggestions about how to resolve them.

---

## Device Won't Show Up in the Port List

Possible causes:

- USB cable is **power-only** (try a known good data cable).
- Board not powered, physically damaged. or MicroSD card not inserted properly.

Checklist:

1. Try a different cable and USB port.
2. Make sure you see a green LED and an orange LED on the board.
3. Check your Device Manager for a new COM port that appears when you plug in the board.

---

TODO: eventually add a table for LED error codes.

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