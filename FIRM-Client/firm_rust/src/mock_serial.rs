use firm_core::constants::packet::{
    CRC_SIZE, HEADER_SIZE, IDENTIFIER_SIZE, LENGTH_SIZE, MIN_PACKET_SIZE,
};
use firm_core::framed_packet::FramedPacket;
use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

#[derive(Default)]
struct State {
    /// Bytes written by the device (test) to be read by the client.
    device_to_client: Mutex<VecDeque<u8>>,
    /// Bytes written by the client to be read by the device (test).
    client_to_device: Mutex<VecDeque<u8>>,
    /// Simulated read timeout for the mock serial port.
    timeout: Mutex<Duration>,
}

#[derive(Clone)]
pub struct MockDeviceHandle {
    state: Arc<State>,
    command_buffer: Arc<Mutex<Vec<u8>>>,
}

impl MockDeviceHandle {
    fn new(state: Arc<State>) -> Self {
        Self {
            state,
            command_buffer: Arc::new(Mutex::new(Vec::new())),
        }
    }

    /// Waits for a framed command and returns its identifier, or None on timeout.
    pub fn wait_for_command_identifier(&self, timeout: Duration) -> io::Result<Option<u16>> {
        let deadline = Instant::now() + timeout;
        let mut command_buffer = self.command_buffer.lock().unwrap();

        loop {
            {
                let mut queue = self.state.client_to_device.lock().unwrap();

                while let Some(byte) = queue.pop_front() {
                    command_buffer.push(byte);
                }
            }

            if command_buffer.len() >= MIN_PACKET_SIZE {
                let len_start = HEADER_SIZE + IDENTIFIER_SIZE;
                if command_buffer.len() >= len_start + LENGTH_SIZE {
                    let payload_len = u32::from_le_bytes(
                        command_buffer[len_start..len_start + LENGTH_SIZE]
                            .try_into()
                            .unwrap(),
                    ) as usize;
                    let frame_len =
                        HEADER_SIZE + IDENTIFIER_SIZE + LENGTH_SIZE + payload_len + CRC_SIZE;
                    if command_buffer.len() >= frame_len {
                        let frame = FramedPacket::from_bytes(&command_buffer[..frame_len])
                            .map_err(|e| {
                                io::Error::new(io::ErrorKind::InvalidData, format!("{e:?}"))
                            })?;
                        command_buffer.drain(..frame_len);
                        return Ok(Some(frame.identifier()));
                    }
                }
            }

            if Instant::now() >= deadline {
                return Ok(None);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    /// Injects one framed data packet into the client's read stream.
    pub fn inject_framed_packet(&self, mocked_packet: FramedPacket) {
        let bytes = mocked_packet.to_bytes();
        let mut queue = self.state.device_to_client.lock().unwrap();
        queue.extend(bytes);
    }
}

#[derive(Clone)]
pub struct MockSerialPort {
    state: Arc<State>,
}

impl MockSerialPort {
    /// Creates a paired mock serial port and device handle.
    pub fn pair(timeout: Duration) -> (Box<dyn SerialPort>, MockDeviceHandle) {
        let state = Arc::new(State {
            timeout: Mutex::new(timeout),
            ..Default::default()
        });
        let port: Box<dyn SerialPort> = Box::new(Self {
            state: state.clone(),
        });
        (port, MockDeviceHandle::new(state))
    }

    fn timeout(&self) -> Duration {
        *self.state.timeout.lock().unwrap()
    }
}

impl Read for MockSerialPort {
    fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        let mut queue = self.state.device_to_client.lock().unwrap();
        if queue.is_empty() {
            drop(queue);
            std::thread::sleep(self.timeout());
            return Err(io::Error::new(io::ErrorKind::TimedOut, "mock timeout"));
        }
        let mut n = 0usize;
        while n < out.len() {
            let Some(byte) = queue.pop_front() else { break };
            out[n] = byte;
            n += 1;
        }
        Ok(n)
    }
}

impl Write for MockSerialPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut queue = self.state.client_to_device.lock().unwrap();
        queue.extend(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl SerialPort for MockSerialPort {
    fn name(&self) -> Option<String> {
        Some("mock".to_string())
    }
    fn baud_rate(&self) -> serialport::Result<u32> {
        Ok(2_000_000)
    }
    fn data_bits(&self) -> serialport::Result<DataBits> {
        Ok(DataBits::Eight)
    }
    fn flow_control(&self) -> serialport::Result<FlowControl> {
        Ok(FlowControl::None)
    }
    fn parity(&self) -> serialport::Result<Parity> {
        Ok(Parity::None)
    }
    fn stop_bits(&self) -> serialport::Result<StopBits> {
        Ok(StopBits::One)
    }
    fn timeout(&self) -> Duration {
        self.timeout()
    }
    fn set_timeout(&mut self, timeout: Duration) -> serialport::Result<()> {
        *self.state.timeout.lock().unwrap() = timeout;
        Ok(())
    }
    fn set_baud_rate(&mut self, _baud_rate: u32) -> serialport::Result<()> {
        Ok(())
    }
    fn set_data_bits(&mut self, _data_bits: DataBits) -> serialport::Result<()> {
        Ok(())
    }
    fn set_flow_control(&mut self, _flow_control: FlowControl) -> serialport::Result<()> {
        Ok(())
    }
    fn set_parity(&mut self, _parity: Parity) -> serialport::Result<()> {
        Ok(())
    }
    fn set_stop_bits(&mut self, _stop_bits: StopBits) -> serialport::Result<()> {
        Ok(())
    }
    fn write_request_to_send(&mut self, _level: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn write_data_terminal_ready(&mut self, _level: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        Ok(true)
    }
    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
        Ok(true)
    }
    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        Ok(true)
    }
    fn bytes_to_read(&self) -> serialport::Result<u32> {
        Ok(self.state.device_to_client.lock().unwrap().len() as u32)
    }
    fn bytes_to_write(&self) -> serialport::Result<u32> {
        Ok(0)
    }
    fn clear(&self, _buffer_to_clear: ClearBuffer) -> serialport::Result<()> {
        Ok(())
    }
    fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
        Ok(Box::new(self.clone()))
    }
    fn set_break(&self) -> serialport::Result<()> {
        Ok(())
    }
    fn clear_break(&self) -> serialport::Result<()> {
        Ok(())
    }
}
