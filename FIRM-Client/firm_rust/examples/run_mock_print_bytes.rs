use firm_core::client_packets::{FIRMCommandPacket, FIRMLogPacket};
use firm_core::constants::log_parsing::{FIRMLogPacketType, HEADER_PARSE_DELAY, HEADER_TOTAL_SIZE};
use firm_core::data_parser::SerialParser;
use firm_core::framed_packet::Framed;
use firm_core::log_parsing::LogParser;
use serialport::SerialPort;
use std::fs::File;
use std::io::{Read, Write};
use std::process::ExitCode;
use std::time::{Duration, Instant};

// ---- Hardcoded settings (edit these) ----
const PORT: &str = "COM12";
const LOG_PATH: &str = r"C:\Users\jackg\Downloads\LOG1.TXT";
const BAUD_RATE: u32 = 2_000_000;
// IMPORTANT: A long read timeout will slow streaming because we poll RX often.
// Keep this very small so RX polling doesn't block sending.
const TIMEOUT_SECONDS: f64 = 0.001;

// If true, pace the stream based on log timestamps. If your log is long, this will take
// approximately (log duration / SPEED) to finish.
// Set REALTIME=false to send as fast as possible.
const REALTIME: bool = true;
const SPEED: f64 = 1.0;
const CHUNK_SIZE: usize = 80_000;
const DRAIN_SECONDS: f64 = 1.0;
const STATUS_INTERVAL: Duration = Duration::from_millis(250);
const PRINT_RX_BYTES: bool = true;
// ----------------------------------------

fn print_hex(prefix: &str, bytes: &[u8]) {
    if bytes.is_empty() {
        return;
    }

    let hex = bytes
        .iter()
        .map(|b| format!("{:02X}", b))
        .collect::<Vec<_>>()
        .join(" ");
    println!("{prefix}{hex}");
}

fn read_and_count_nonblocking(
    port: &mut dyn SerialPort,
    buffer: &mut [u8],
    parser: &mut SerialParser,
    total_data_packets: &mut u64,
) {
    // Avoid blocking on reads when no data is available.
    // With a non-zero port timeout, `read()` can wait up to TIMEOUT_SECONDS; since we poll RX a
    // lot while streaming, that can drastically slow the mock.
    if let Ok(0) = port.bytes_to_read() {
        return;
    }

    match port.read(buffer) {
        Ok(n @ 1..) => {
            if PRINT_RX_BYTES {
                print_hex("\n\n", &buffer[..n]);
            }
            parser.parse_bytes(&buffer[..n]);
            while parser.get_data_packet().is_some() {
                *total_data_packets += 1;
            }
            // Drain responses too so the internal parser doesn't grow unbounded.
            while parser.get_response_packet().is_some() {}
        }
        Ok(_) => {}
        Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
        Err(e) => eprintln!("Serial read error: {e}"),
    }
}

fn read_and_count_for_duration(
    port: &mut dyn SerialPort,
    buffer: &mut [u8],
    parser: &mut SerialParser,
    total_data_packets: &mut u64,
    duration: Duration,
    last_status: &mut Instant,
) {
    let start = Instant::now();
    while start.elapsed() < duration {
        read_and_count_nonblocking(port, buffer, parser, total_data_packets);

        if last_status.elapsed() >= STATUS_INTERVAL {
            *last_status = Instant::now();
            println!("RX FIRMDataPackets: {total_data_packets}");
        }

        // Avoid a tight spin when the device is quiet.
        std::thread::sleep(Duration::from_millis(1));
    }
}

fn main() -> ExitCode {
    if SPEED <= 0.0 {
        eprintln!("--speed must be > 0");
        return ExitCode::FAILURE;
    }

    let mut port = match serialport::new(PORT, BAUD_RATE)
        .timeout(Duration::from_secs_f64(TIMEOUT_SECONDS))
        .open()
    {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Failed to open {}: {e}", PORT);
            return ExitCode::FAILURE;
        }
    };

    let mut rx_buf = [0u8; 4096];
    let mut rx_parser = SerialParser::new();
    let mut rx_data_packets: u64 = 0;
    let mut last_status = Instant::now();

    // Start mock mode.
    let mock_cmd = FIRMCommandPacket::build_mock_command().to_bytes();
    if let Err(e) = port.write_all(&mock_cmd) {
        eprintln!("Failed to write mock command: {e}");
        return ExitCode::FAILURE;
    }
    let _ = port.flush();

    // Read whatever the device responds with for a short time (ack, etc.).
    read_and_count_for_duration(
        &mut *port,
        &mut rx_buf,
        &mut rx_parser,
        &mut rx_data_packets,
        Duration::from_millis(200),
        &mut last_status,
    );

    // Stream the mock log.
    let mut file = match File::open(LOG_PATH) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("Failed to open log file {}: {e}", LOG_PATH);
            return ExitCode::FAILURE;
        }
    };

    let mut header = vec![0u8; HEADER_TOTAL_SIZE];
    if let Err(e) = file.read_exact(&mut header) {
        eprintln!("Failed to read log header: {e}");
        return ExitCode::FAILURE;
    }

    // Send header packet.
    let header_packet = FIRMLogPacket::new(FIRMLogPacketType::HeaderPacket, header.clone());
    if let Err(e) = port.write_all(&header_packet.to_bytes()) {
        eprintln!("Failed to write header packet: {e}");
        return ExitCode::FAILURE;
    }
    let _ = port.flush();

    // Give device time to parse header (but keep counting RX data packets).
    read_and_count_for_duration(
        &mut *port,
        &mut rx_buf,
        &mut rx_parser,
        &mut rx_data_packets,
        HEADER_PARSE_DELAY,
        &mut last_status,
    );

    let mut parser = LogParser::new();
    parser.read_header(&header);

    let mut buf = vec![0u8; CHUNK_SIZE];
    let mut packets_sent = 0usize;

    let stream_start = Instant::now();
    let mut total_delay_seconds = 0.0f64;

    loop {
        // Keep counting any incoming packets.
        read_and_count_nonblocking(
            &mut *port,
            &mut rx_buf,
            &mut rx_parser,
            &mut rx_data_packets,
        );

        if last_status.elapsed() >= STATUS_INTERVAL {
            last_status = Instant::now();
            println!("RX FIRMDataPackets: {rx_data_packets}");
        }

        let n = match file.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => n,
            Err(e) => {
                eprintln!("Failed to read log file: {e}");
                return ExitCode::FAILURE;
            }
        };

        parser.parse_bytes(&buf[..n]);

        while let Some((packet, delay_seconds)) = parser.get_packet_and_time_delay() {
            // Send next mock packet.
            if let Err(e) = port.write_all(&packet.to_bytes()) {
                eprintln!("Failed to write mock packet: {e}");
                return ExitCode::FAILURE;
            }
            packets_sent += 1;

            // Count anything we receive as we go.
            read_and_count_nonblocking(
                &mut *port,
                &mut rx_buf,
                &mut rx_parser,
                &mut rx_data_packets,
            );

            if REALTIME && delay_seconds > 0.0 {
                total_delay_seconds += delay_seconds;

                let stream_elapsed = stream_start.elapsed().as_secs_f64();
                // Only wait if we're not already behind.
                let target_elapsed = total_delay_seconds / SPEED;
                if stream_elapsed <= target_elapsed {
                    let wait_s = (target_elapsed - stream_elapsed).max(0.0);
                    read_and_count_for_duration(
                        &mut *port,
                        &mut rx_buf,
                        &mut rx_parser,
                        &mut rx_data_packets,
                        Duration::from_secs_f64(wait_s),
                        &mut last_status,
                    );
                }
            }
        }

        if parser.eof_reached() {
            break;
        }
    }

    // Drain any remaining packets buffered by the parser.
    while let Some((packet, delay_seconds)) = parser.get_packet_and_time_delay() {
        if let Err(e) = port.write_all(&packet.to_bytes()) {
            eprintln!("Failed to write mock packet: {e}");
            return ExitCode::FAILURE;
        }
        packets_sent += 1;

        read_and_count_nonblocking(
            &mut *port,
            &mut rx_buf,
            &mut rx_parser,
            &mut rx_data_packets,
        );

        if REALTIME && delay_seconds > 0.0 {
            total_delay_seconds += delay_seconds;
            let stream_elapsed = stream_start.elapsed().as_secs_f64();
            let target_elapsed = total_delay_seconds / SPEED;
            if stream_elapsed <= target_elapsed {
                let wait_s = (target_elapsed - stream_elapsed).max(0.0);
                read_and_count_for_duration(
                    &mut *port,
                    &mut rx_buf,
                    &mut rx_parser,
                    &mut rx_data_packets,
                    Duration::from_secs_f64(wait_s),
                    &mut last_status,
                );
            }
        }
    }

    // Send cancel (fire-and-forget) to exit mock mode.
    let cancel_cmd = FIRMCommandPacket::build_cancel_command().to_bytes();
    let _ = port.write_all(&cancel_cmd);
    let _ = port.flush();

    eprintln!(
        "Sent {packets_sent} mock packets; draining... (RX FIRMDataPackets so far: {rx_data_packets})"
    );
    read_and_count_for_duration(
        &mut *port,
        &mut rx_buf,
        &mut rx_parser,
        &mut rx_data_packets,
        Duration::from_secs_f64(DRAIN_SECONDS.max(0.0)),
        &mut last_status,
    );

    println!("Final RX FIRMDataPackets: {rx_data_packets}");

    ExitCode::SUCCESS
}
