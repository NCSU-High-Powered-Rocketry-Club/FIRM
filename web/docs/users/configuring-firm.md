# Using FIRM - Configuring Your Device

This page explains how to **configure settings on FIRM** using the web app and a USB connection.

We assume you've already:

- Connected the device to your computer, and
- Verified it is successfully communnicating (see **[Quick Start](quick-start.md)**).

---

## 🏷️ Device Name

This is simply the label FIRM reports to the app.  
You can name it whatever helps you keep track of your device, for example:

- "FIRM Flight Computer"
- "Payload FIRM"
- "Airbrakes FC"

There's no functional impact - it's just for organization.

---

## 📝 Data Packet Fields

TODO: this needs to be updated once sensor data is finalized

FIRM is records/calculates many fields, by default, it sends over:

- Accel X, Y, Z
- Gyro X, Y, Z
- Pressure
- Altitude

But there are additional fields available:

TODO: do a table here showing what each field does/means

You can select as many or as few as you want.

**Recommended:**

- Using the default settings is fine for most applications.
- Alternatively, you can choose to **disable** any fields you don't need--FIRM keeps a local copy of all data it
  records/calculates, so you don't have to be worried about losing any data.

> You can also **import or export** your entire configuration as a JSON file using the buttons at the bottom.

---

## 〰️ Update Frequency (1-500 Hz)

This controls how often FIRM sends sensor updates to the web app.

- **Higher = smoother, more detailed data.**
- **Lower = lighter communication load, helpful if your device can't keep up with the rate of updates.**

General guidance:

- **100 Hz** → smooth real-time graphs, great for most tests
- **200-500 Hz** → high-performance data (only needed for special use cases)
- **20-50 Hz** → slow tests or debugging

**Recommended:**  
Use **100 Hz**, unless you specifically need a higher rate.

---

## 🔗 Protocol

This tells the web app how to talk to the device.

There are other protocols that can be used such as I2C, SPI, and UART, but USB is the easiest to set up.

**Recommended:**  
Leave this set to **USB**.

---

## 🔄 Apply / Reset

After making changes:

- **Apply Settings** writes the configuration to the device.
- **Reset** restores all fields back to their defaults.

Make sure to save a copy of your settings once you're done!

---

If you want to run a test or actually start logging data, continue to  
➡️ **[Logging & Downloading Data](data-logging.md)**
