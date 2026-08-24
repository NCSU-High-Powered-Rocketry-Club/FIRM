use anyhow::{Context, Result};
use clap::Parser;
use firm_core::constants::command::{
    NUMBER_OF_CALIBRATION_OFFSETS, NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS,
};
use firm_rust::FIRMClient;
use std::process::ExitCode;
use std::time::Duration;

// cargo run -p firm_rust --example test_calibration -- --port COM12

#[derive(Parser, Debug)]
#[command(about = "Reset calibration to identity, then run magnetometer calibration")]
struct Args {
    /// Serial port name (e.g. COM12). If omitted, uses the first detected port.
    #[arg(long)]
    port: Option<String>,

    /// Baud rate for the device.
    #[arg(long, default_value_t = 2_000_000)]
    baud: u32,

    /// Serial read timeout in seconds.
    #[arg(long, default_value_t = 0.1)]
    timeout_s: f64,

    /// How long to collect magnetometer samples before solving.
    #[arg(long, default_value_t = 30)]
    collect_seconds: u64,

    /// Timeout (ms) for each "apply calibration" command acknowledgement.
    #[arg(long, default_value_t = 750)]
    apply_timeout_ms: u64,

    /// Timeout (ms) for querying the current calibration.
    #[arg(long, default_value_t = 750)]
    query_timeout_ms: u64,
}

fn identity_matrix_row_major() -> [f32; NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS] {
    let mut m = [0.0f32; NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS];

    // Expected 3x3 row-major: [m11 m12 m13 m21 m22 m23 m31 m32 m33]
    // Fill with identity.
    if NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS >= 1 {
        m[0] = 1.0;
    }
    if NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS >= 5 {
        m[4] = 1.0;
    }
    if NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS >= 9 {
        m[8] = 1.0;
    }

    m
}

fn pick_port(port: Option<String>) -> Result<String> {
    if let Some(p) = port {
        return Ok(p);
    }

    let ports = serialport::available_ports().context("No serial ports found")?;
    let first = ports
        .first()
        .context("No serial ports detected")?
        .port_name
        .clone();
    Ok(first)
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("{e:?}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<()> {
    let args = Args::parse();
    let port_name = pick_port(args.port)?;

    println!("Connecting to {port_name} @ {} baud", args.baud);

    let mut client = FIRMClient::new(&port_name, args.baud, args.timeout_s)
        .with_context(|| format!("Failed to open serial port {port_name}"))?;
    client.start();

    let offsets = [0.0f32; NUMBER_OF_CALIBRATION_OFFSETS];
    let identity = identity_matrix_row_major();

    let apply_timeout = Duration::from_millis(args.apply_timeout_ms);
    let query_timeout = Duration::from_millis(args.query_timeout_ms);

    println!("Resetting magnetometer calibration to identity...");
    match client.set_magnetometer_calibration(offsets, identity, apply_timeout)? {
        Some(true) => println!("Magnetometer calibration reset ✅"),
        Some(false) => anyhow::bail!("Device rejected magnetometer calibration reset"),
        None => anyhow::bail!("Timed out waiting for magnetometer calibration reset ack"),
    }

    println!("Resetting IMU calibration (accel+gyro) to identity...");
    match client.set_imu_calibration(offsets, identity, offsets, identity, apply_timeout)? {
        Some(true) => println!("IMU calibration reset ✅"),
        Some(false) => anyhow::bail!("Device rejected IMU calibration reset"),
        None => anyhow::bail!("Timed out waiting for IMU calibration reset ack"),
    }

    if let Some(cal) = client.get_calibration(query_timeout)? {
        println!("Device calibration after reset: {cal:#?}");
    } else {
        println!("(No GetCalibration response within timeout; continuing.)");
    }

    let collection_duration = Duration::from_secs(args.collect_seconds);
    println!(
        "Starting magnetometer calibration collection for {}s... rotate device now",
        args.collect_seconds
    );

    let apply_ok =
        client.run_and_apply_magnetometer_calibration(collection_duration, apply_timeout)?;

    match apply_ok {
        Some(true) => println!("Magnetometer calibration applied ✅"),
        Some(false) => anyhow::bail!("Device rejected magnetometer calibration apply"),
        None => anyhow::bail!("Calibration failed (not enough data?) or no ack"),
    }

    if let Some(cal) = client.get_calibration(query_timeout)? {
        println!("Device calibration after apply: {cal:#?}");
    } else {
        println!("(No GetCalibration response within timeout.)");
    }

    client.stop();
    Ok(())
}
