use clap::Parser;
use firm_core::client_packets::FIRMLogPacket;
use firm_core::constants::log_parsing::FIRMLogPacketType;
use firm_core::constants::log_parsing::HEADER_TOTAL_SIZE;
use firm_core::framed_packet::Framed;
use firm_core::log_parsing::LogParser;
use std::fs::File;
use std::io::Read;
use std::process::ExitCode;

#[derive(Parser, Debug)]
#[command()]
struct Args {
    /// Path to the log file to verify
    log_path: String,

    /// Size of file chunks to read at a time
    #[arg(short, long, default_value_t = 100_000)]
    chunk_size: usize,
}

fn main() -> ExitCode {
    let args = Args::parse();
    let mut parser = LogParser::new();

    let mut file = File::open(&args.log_path).expect("Failed to open log file");

    let mut header: Vec<u8> = vec![0u8; HEADER_TOTAL_SIZE];
    file.read_exact(&mut header).expect("Failed to read header");

    if header.starts_with(b"FIRM") {
        println!("✅ File header looks correct: FIRM...");
    } else {
        println!("❌ WRONG FILE! Header starts with: {:02X?}", &header[0..4]);
        return ExitCode::FAILURE;
    }

    parser.read_header(&header);

    let mut buf: Vec<u8> = vec![0u8; args.chunk_size];

    let mut count_total = 0usize;
    let mut count_bmp = 0usize;
    let mut count_imu = 0usize;
    let mut count_mag = 0usize;

    let mut total_delay = 0.0f64;

    loop {
        let n = file.read(&mut buf).expect("Failed to read from file");
        if n == 0 {
            break;
        }

        parser.parse_bytes(&buf[..n]);

        // Just verifies the round-trip serialization/parsing of packets
        while let Some((pkt, delay_s)) = parser.get_packet_and_time_delay() {
            total_delay += delay_s;
            let bytes = pkt.to_bytes();
            let parsed = FIRMLogPacket::from_bytes(&bytes)
                .expect("failed to parse bytes we just serialized (header/len/crc mismatch)");
            assert_eq!(parsed.payload(), pkt.payload());

            count_total += 1;
            match parsed.packet_type() {
                FIRMLogPacketType::BarometerPacket => count_bmp += 1,
                FIRMLogPacketType::IMUPacket => count_imu += 1,
                FIRMLogPacketType::MagnetometerPacket => count_mag += 1,
                other => println!("Unexpected packet type: {other:?}"),
            }

            if count_total <= 5 {
                let payload_len = parsed.payload().len();
                let id_char = parsed.packet_type().as_char();

                println!(
                    "#{count_total} id={id_char} payload_len={payload_len} delay_s={delay_s:.6}",
                );
            }
        }
    }

    println!(
        "OK: total={count_total} B={count_bmp} I={count_imu} M={count_mag} delay_s={total_delay:.6} (round-trip header/len/crc verified)"
    );

    ExitCode::SUCCESS
}
