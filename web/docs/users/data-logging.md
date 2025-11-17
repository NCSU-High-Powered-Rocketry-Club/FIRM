# Using FIRM - Logging & Downloading Data

Once your device is configured, the next step is to **log data** and **get it off the device** for analysis.

This page focuses on:

- Running a test or flight
- Downloading the raw FIRM data afterwards
- How to stream and read FIRM data to your own devices

---

## How Logging Works

FIRM is designed to keep things as simple as possible:

- **As soon as the board is powered on, it boots up and immediately starts logging data** into a new log file.
- There is **no "start" or "stop" button** - FIRM just records until it powers off.
- While logging internally, FIRM also **streams data** to whatever computer (Raspberry Pi, Arduino, etc.) it's connected
  to.

> 📚 Make sure you have configured the device using the web app to use the correct protocol and fields before continuing.

This means that you can:

- Download the full “raw device log” later _even if no computer was connected_.
- Still read/stream real-time data if a computer _is_ connected.

---

## Downloading the Raw FIRM Device Log File

After a test or flight:

1. Plug your FIRM device into your laptop.
2. Open the FIRM Web App at **https://firm.ncsurocketry.org/**
3. Connect to the device using the **Connect to FIRM** button.
4. Scroll down to the **Download Log File** section.

You'll see something like this:

![Download Log UI](../assets/download-log.png)

Use the dropdown to pick which recorded session you want (usually just _Most Recent Session_), then click:

**Download Log**

This gives you a `.csv` log file containing what the device recorded internally.

> You can also **import/export device settings** as JSON files using the import and export buttons.

---

## Streaming Data to Other Devices (Python, JS/TS, Rust, Arduino, etc.)

While the device is logging internally, it also **streams data** to whatever system it's plugged into.

We've built a cross-language client library to make this painless. Currently, we have support for:

- **[Python (PyPI)](https://pypi.org/project/firm-hprc/)**
- **[JavaScript / TypeScript (NPM)](https://www.npmjs.com/package/firm-client)**
- **Rust crate**
- **Arduino library (planned)**

All of these are maintained in this repo:  
https://github.com/NCSU-High-Powered-Rocketry-Club/FIRM-Client

### What this means for you

As long as you:

1. Configure the FIRM device using the web app, and
2. Plug it into something capable of reading serial data (a Raspberry Pi, a laptop, an Arduino, a flight computer, etc.),

...then all you need to do is follow the [README](https://github.com/NCSU-High-Powered-Rocketry-Club/FIRM-Client) in the **FIRM-Client** repo to start reading parsed FIRM data in your code.

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
[//]: # '- You can download the "raw device log" anytime using the web app.'
[//]: # "- FIRM streams live data to any connected computer."
[//]: # "- Use the **FIRM-Client** library to read data in Python, JS/TS, Rust, or (soon) Arduino."
[//]: # "- The guide for using the client library is in the other repo; this page just explains how FIRM produces data."
