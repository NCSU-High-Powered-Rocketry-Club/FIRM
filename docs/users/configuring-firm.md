# Using FIRM - Configuring Your Device

This page explains how to **configure settings on FIRM** using the web app and a USB connection.

We assume you've already:

- Connected the device to your computer, and
- Verified it is successfully communicating (see **[Quick Start](quick-start.md)**).

---

## 🏷️ Device Name

This is simply the label FIRM reports to the app.  
You can name it whatever helps you keep track of your device, for example:

- "FIRM Flight Computer"
- "Payload FIRM"
- "Airbrakes FC"

There's no functional impact--it's just for organization.

---

### 📝 Data Packet Fields

FIRM sends a flat data packet: every data point is its own top-level field rather than a nested
structure. Every packet contains all of the fields below.

#### Sent by the device

| Field Name                   | Units | Description                                |
| ---------------------------- | ----- | ------------------------------------------ |
| timestamp_seconds            | s     | Monotonic device timestamp.                |
| temperature_celsius          | °C    | Barometer temperature.                     |
| pressure_pascals             | Pa    | Barometric pressure.                       |
| raw_acceleration_x_gs        | g     | IMU accelerometer X axis.                  |
| raw_acceleration_y_gs        | g     | IMU accelerometer Y axis.                  |
| raw_acceleration_z_gs        | g     | IMU accelerometer Z axis.                  |
| raw_angular_rate_x_deg_per_s | °/s   | Gyroscope X axis.                          |
| raw_angular_rate_y_deg_per_s | °/s   | Gyroscope Y axis.                          |
| raw_angular_rate_z_deg_per_s | °/s   | Gyroscope Z axis.                          |
| magnetic_field_x_microteslas | µT    | Magnetometer X axis.                       |
| magnetic_field_y_microteslas | µT    | Magnetometer Y axis.                       |
| magnetic_field_z_microteslas | µT    | Magnetometer Z axis.                       |
| high_g_accel_x_gs            | g     | High-g accelerometer X axis.               |
| high_g_accel_y_gs            | g     | High-g accelerometer Y axis.               |
| high_g_accel_z_gs            | g     | High-g accelerometer Z axis.               |
| est_position_z_meters        | m     | Filter-estimated vertical position.\*      |
| est_velocity_z_meters_per_s  | m/s   | Filter-estimated vertical velocity.\*      |
| est_quaternion_w/x/y/z       | —     | Filter-estimated orientation quaternion.\* |

\* The on-device filter does not populate these fields yet, so they are currently always 0.

#### Derived by the client library

| Field Name                        | Units | Description                                  |
| --------------------------------- | ----- | -------------------------------------------- |
| raw_rotated_acceleration_x/y/z_gs | g     | Acceleration rotated into the world frame.\* |
| est_tilt_angle_degrees            | °     | Tilt angle from the orientation.\*           |
| est_mach_number                   | —     | Mach number from velocity and temperature.\* |

\* These are computed from the filter estimates above, so they are not meaningful until the device populates
them.

> You can **import or export** the device name, frequency, and protocol as a JSON file using the
> buttons at the bottom of the settings panel.

---

## 〰️ Update Frequency (1-1000 Hz)

This controls how often FIRM sends data packets. **100 Hz** is usually fine for most applications.

> A new frequency is saved right away but takes effect the next time FIRM powers on.

- **Higher = smoother, more detailed data.**
- **Lower = lighter communication load, helpful if your device can't keep up with the rate of updates.**

General guidance:

- **100 Hz** → smooth real-time graphs, great for most tests
- **200-1000 Hz** → high-performance data (only needed for special use cases)
- **20-50 Hz** → slow tests or debugging

**Recommended:**  
Use **100 Hz**, unless you specifically need a higher rate.

---

## 🔗 Protocol

This setting is stored on the device. FIRM currently only sends data over **USB**, and the web app
always talks to it over USB; the I2C, SPI, and UART options are reserved for future use.

**Recommended:**  
Leave this set to **USB**.

---

## 🔄 Apply / Import / Export

After making changes:

- **Apply Settings** writes the configuration to the device.
- **Export** saves the current form as a JSON file, and **Import** loads one back into the form
  (press **Apply Settings** afterwards to write it to the device).

Make sure to save a copy of your settings once you're done!

---

If you want to run a test or actually start logging data, continue to  
➡️ **[Logging & Downloading Data](data-logging.md)**
