use anyhow::{Context, Result};
use clap::Parser;
use firm_core::calibration::MagnetometerCalibrator;
use std::fs::File;
use std::io::{BufRead, BufReader};

// cargo run -p firm_rust --example calibrate_mag_csv_range -- "C:\Users\jackg\Desktop\C or C++ Projects\FIRM\FIRM\python\scripts\MMC5983MA_data.csv" --start 1000 --end 10000

#[derive(Parser, Debug)]
#[command(about = "Run magnetometer calibration on a CSV capture over a sample index range")]
struct Args {
    /// Path to the CSV file.
    csv_path: String,

    /// Start sample index (0-based) within the magnetometer data section.
    #[arg(long, default_value_t = 0)]
    start: usize,

    /// End sample index (0-based, inclusive) within the magnetometer data section.
    #[arg(long)]
    end: Option<usize>,
}

fn parse_mag_csv(path: &str) -> Result<Vec<[f32; 3]>> {
    let file = File::open(path).with_context(|| format!("Failed to open CSV file: {path}"))?;
    let reader = BufReader::new(file);

    let mut in_data = false;
    let mut mags: Vec<[f32; 3]> = Vec::new();

    for line in reader.lines() {
        let line = line?;
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }

        if !in_data {
            // Find the start of the data table.
            // Expected header: timestamp,mag_x,mag_y,mag_z
            let lower = trimmed.to_ascii_lowercase();
            if lower.starts_with("timestamp") && lower.contains("mag_x") && lower.contains("mag_y")
            {
                in_data = true;
            }
            continue;
        }

        // Parse data row: timestamp,mag_x,mag_y,mag_z
        let mut parts = trimmed.split(',');
        let _timestamp = parts.next();
        let x = parts.next();
        let y = parts.next();
        let z = parts.next();

        let (Some(x), Some(y), Some(z)) = (x, y, z) else {
            continue;
        };

        let x: f32 = match x.trim().parse() {
            Ok(v) => v,
            Err(_) => continue,
        };
        let y: f32 = match y.trim().parse() {
            Ok(v) => v,
            Err(_) => continue,
        };
        let z: f32 = match z.trim().parse() {
            Ok(v) => v,
            Err(_) => continue,
        };

        mags.push([x, y, z]);
    }

    if mags.is_empty() {
        anyhow::bail!(
            "No magnetometer samples found. Expected a data header like: timestamp,mag_x,mag_y,mag_z"
        );
    }

    Ok(mags)
}

fn main() -> Result<()> {
    let args = Args::parse();

    let mags = parse_mag_csv(&args.csv_path)?;
    let total = mags.len();

    let start = args.start;
    let end_inclusive = args.end.unwrap_or_else(|| total.saturating_sub(1));

    if start >= total {
        anyhow::bail!("start index {start} is out of bounds (total samples: {total})");
    }
    if end_inclusive >= total {
        anyhow::bail!("end index {end_inclusive} is out of bounds (total samples: {total})");
    }
    if start > end_inclusive {
        anyhow::bail!("start index {start} must be <= end index {end_inclusive}");
    }

    let used = end_inclusive - start + 1;

    let mut calibrator = MagnetometerCalibrator::new();
    calibrator.start();

    for [x, y, z] in &mags[start..=end_inclusive] {
        calibrator.add_sample_xyz(*x, *y, *z);
    }

    calibrator.stop();

    let cal = calibrator
        .calculate()
        .context("Calibration failed (insufficient motion coverage or ill-conditioned fit)")?;

    let (offsets, matrix_m_row_major) = cal.to_arrays();

    // MATLAB row-vector form uses A = M^T where device uses: c = M * (raw - b)
    let a = [
        matrix_m_row_major[0],
        matrix_m_row_major[3],
        matrix_m_row_major[6],
        matrix_m_row_major[1],
        matrix_m_row_major[4],
        matrix_m_row_major[7],
        matrix_m_row_major[2],
        matrix_m_row_major[5],
        matrix_m_row_major[8],
    ];

    println!("Loaded {total} mag samples from CSV");
    println!("Using range [{start}..{end_inclusive}] ({used} samples)");

    println!(
        "\nCalibration values for device (M * (raw - b)):\n  magnetometer_offsets: [{:.6}, {:.6}, {:.6}]\n  magnetometer_scale_matrix: [{:.9}, {:.9}, {:.9}, {:.9}, {:.9}, {:.9}, {:.9}, {:.9}, {:.9}]",
        offsets[0],
        offsets[1],
        offsets[2],
        matrix_m_row_major[0],
        matrix_m_row_major[1],
        matrix_m_row_major[2],
        matrix_m_row_major[3],
        matrix_m_row_major[4],
        matrix_m_row_major[5],
        matrix_m_row_major[6],
        matrix_m_row_major[7],
        matrix_m_row_major[8]
    );

    println!(
        "\nMATLAB magcal-style (C = (mag - b) * A):\n  b = [{:.6}, {:.6}, {:.6}]\n  A = [{:.6}, {:.6}, {:.6}; {:.6}, {:.6}, {:.6}; {:.6}, {:.6}, {:.6}]",
        offsets[0], offsets[1], offsets[2], a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8]
    );

    Ok(())
}
