use std::{process::exit, time::Duration};

use firm_rs::parser::SerialParser;

fn main() {
    let ports = serialport::available_ports().expect("No ports found!");

    if ports.is_empty() {
        eprintln!("No serial ports detected");
        exit(1);
    }

    if ports.len() > 1 {
        eprintln!("Too many serial ports detected");
        exit(1);
    }

    let port_info: &serialport::SerialPortInfo = &ports[0];

    let mut port = serialport::new(port_info.port_name.clone(), 115_200)
        .timeout(Duration::from_millis(10))
        .open()
        .expect("Failed to open port");

    let mut parser = SerialParser::new();

    loop {
        let mut buf = [0; 1024];
        let num_bytes = port.read(&mut buf).unwrap_or(0);

        if num_bytes > 0 {
            let slice = &buf[0..num_bytes];
            parser.parse_bytes(slice);
        }

        while let Some(p) = parser.get_packet() {
            println!("{p:#?}");
        }
    }
}
