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

There's no functional impact--it's just for organization.

---

### 📝 Data Packet Fields

FIRM generates a flat data packet, so every individual data point is sent as its own top-level field rather than nested structures. This makes it easy to enable/disable individual data points.

Below is a list of all available fields, grouped by type:

#### Raw Sensor Data

| Field Name            | Units   | Description                 |
| --------------------- | ------- | --------------------------- |
| timestamp_seconds     | seconds | Monotonic device timestamp. |
| accel_x_meters_per_s2 | m/s²    | Raw accelerometer X axis.   |
| accel_y_meters_per_s2 | m/s²    | Raw accelerometer Y axis.   |
| accel_z_meters_per_s2 | m/s²    | Raw accelerometer Z axis.   |
| gyro_x_radians_per_s  | rad/s   | Raw gyro X axis.            |
| gyro_y_radians_per_s  | rad/s   | Raw gyro Y axis.            |
| gyro_z_radians_per_s  | rad/s   | Raw gyro Z axis.            |
| mag_x_microteslas     | µT      | Raw magnetometer X axis.    |
| mag_y_microteslas     | µT      | Raw magnetometer Y axis.    |
| mag_z_microteslas     | µT      | Raw magnetometer Z axis.    |
| pressure_pascals      | Pa      | Raw barometric pressure.    |
| temperature_celsius   | °C      | Raw temperature reading.    |

#### Estimated (Kalman Filtered) Data

| Field Name              | Units | Description                          |
| ----------------------- | ----- | ------------------------------------ |
| pos_x_meters            | m     | Estimated position X.                |
| pos_y_meters            | m     | Estimated position Y.                |
| pos_z_meters            | m     | Estimated position Z.                |
| vel_x_meters_per_s      | m/s   | Estimated velocity X.                |
| vel_y_meters_per_s      | m/s   | Estimated velocity Y.                |
| vel_z_meters_per_s      | m/s   | Estimated velocity Z.                |
| acc_x_meters_per_s2_kf  | m/s²  | Estimated acceleration X (filtered). |
| acc_y_meters_per_s2_kf  | m/s²  | Estimated acceleration Y (filtered). |
| acc_z_meters_per_s2_kf  | m/s²  | Estimated acceleration Z (filtered). |
| gyro_x_radians_per_s_kf | rad/s | Filter-estimated gyro X.             |
| gyro_y_radians_per_s_kf | rad/s | Filter-estimated gyro Y.             |
| gyro_z_radians_per_s_kf | rad/s | Filter-estimated gyro Z.             |
| quaternion_w            | —     | Orientation quaternion W.            |
| quaternion_x            | —     | Orientation quaternion X.            |
| quaternion_y            | —     | Orientation quaternion Y.            |
| quaternion_z            | —     | Orientation quaternion Z.            |

#### Derived Data

| Field Name                  | Units | Description                           |
| --------------------------- | ----- | ------------------------------------- |
| pitch_radians               | rad   | Derived from quaternion orientation.  |
| roll_radians                | rad   | Derived from quaternion orientation.  |
| yaw_radians                 | rad   | Derived from quaternion orientation.  |
| altitude_meters             | m     | Barometric altitude from pressure.    |
| calculated_pressure_pascals | Pa    | Pressure corrected via KF (optional). |

You can select as many or as few as you want.

**Recommended:**

- Using the default settings is fine for most applications.
- Alternatively, you can choose to **disable** any fields you don't need--FIRM keeps a local copy of all data it
  records, so you don't have to be worried about losing any data.

> You can also **import or export** your entire configuration as a JSON file using the buttons at the bottom.

---

## 〰️ Update Frequency (1-500 Hz)

This controls how often FIRM sends data packets. **100 Hz** is usually fine for most applications.

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
