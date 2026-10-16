# Using FIRM - Logging & Downloading Data

Once your device is configured, the next step is to **log data** and **get it off the device** for analysis.

This page focuses on:

- Running a test or flight
- Downloading the raw FIRM data afterwards
- How to stream and read FIRM data to your own devices

---

## 📜 How Logging Works

FIRM is designed to keep things as simple as possible:

- **As soon as the board is powered on, it boots up and immediately starts logging data** into a new log file.
- There is **no "start" or "stop" button** - FIRM just records until it powers off.
- While logging internally, FIRM also **streams data over USB** to whatever computer (laptop, Raspberry Pi, etc.)
  it's plugged into. USB is currently the only way FIRM sends live data.
- Each log file records the device's settings and calibration from when it powered on. After changing calibration,
  power-cycle FIRM so the next log file records the new values.

This means that you can:

- Download the full "raw device log" later _even if no computer was connected_.
- Still read/stream real-time data if a computer _is_ connected.

---

## ⬇️ Downloading the Raw FIRM Device Log File

Downloading logs through the web app is not available yet. For now, after a test or flight:

1. Unplug FIRM and remove the MicroSD card.
2. Put the card in your computer. Each power-on creates a new file named `logN.frm`, using the lowest
   number that isn't already on the card. If you never delete logs, the highest number is the most recent session;
   after deleting older logs, a new session can reuse a lower number, so check file times or clear the card
   between sessions.
3. Copy the files you want. The `firm-log` tool from the
   [`firm-hprc`](https://pypi.org/project/firm-hprc/) Python package can ingest and decode them.

---

## 🖥️ Connecting FIRM Other Devices

While the device is logging internally, it also **streams data over USB** to whatever system it's plugged into.

We've built a cross-language client library to make this painless. Currently, we have support for:

- **[Python (PyPI)](https://pypi.org/project/firm-client/)**
- **[JavaScript / TypeScript (NPM)](https://www.npmjs.com/package/firm-client)**
- **Rust crate**
- **Arduino library (planned)**

All of these are maintained in the `client/` folder of the FIRM repo:  
**[https://github.com/NCSU-High-Powered-Rocketry-Club/FIRM/tree/main/client](https://github.com/NCSU-High-Powered-Rocketry-Club/FIRM/tree/main/client)**

### 👨‍🚀 What this means for you

As long as you:

1. Configure the FIRM device using the web app, and
2. Plug it into a USB host that can read serial data (a laptop, a Raspberry Pi, a flight computer, etc.),

...then all you need to do is follow the [client README](https://github.com/NCSU-High-Powered-Rocketry-Club/FIRM/blob/main/client/README.md) to start reading parsed FIRM data in your code.

The client library handles:

- Reading the raw byte stream
- Parsing FIRM data packets
- Collecting and giving the data to your device

So the workflow becomes:

- Configure FIRM in the web app
- Plug it into your device
- Install the client library
- Read structured FIRM data in your project

That's it--just plug it into your device and start reading data!

---

[//]: # "## Summary"
[//]: #
[//]: # "- FIRM automatically starts logging when powered."
[//]: # '- You can copy the "raw device log" files off the MicroSD card.'
[//]: # "- FIRM streams live data to any connected computer."
[//]: # "- Use the **firm-client** library to read data in Python, JS/TS, Rust, or (soon) Arduino."
[//]: # "- The guide for using the client library is in the other repo; this page just explains how FIRM produces data."
