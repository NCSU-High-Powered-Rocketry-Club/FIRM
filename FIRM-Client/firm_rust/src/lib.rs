use anyhow::Result;
use firm_core::calibration::{MagnetometerCalibration, MagnetometerCalibrator};
use firm_core::client_packets::{FIRMCommandPacket, FIRMLogPacket};
use firm_core::constants::command::{
    NUMBER_OF_CALIBRATION_OFFSETS, NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS,
};
use firm_core::constants::log_parsing::{FIRMLogPacketType, HEADER_PARSE_DELAY, HEADER_TOTAL_SIZE};
use firm_core::data_parser::SerialParser;
use firm_core::firm_packets::{
    CalibrationValues, DeviceConfig, DeviceInfo, DeviceProtocol, FIRMResponse, ProcessedFIRMData,
};
use firm_core::framed_packet::Framed;
use firm_core::log_parsing::LogParser;
use serialport::SerialPort;
use std::collections::VecDeque;
use std::fs::File;
use std::io::{self, Read, Write};
use std::sync::Arc;
use std::sync::RwLock;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender, channel};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

pub mod mock_serial;

/// Interface to the FIRM Client device.
///
/// # Example:
///
///
/// use firm_rust::FIRMClient;
/// use std::{thread, time::Duration};
///
/// fn main() {
///    let mut client = FIRMClient::new("/dev/ttyUSB0", 2_000_000, 0.1);
///    client.start();
///
///    loop {
///         while let Ok(packet) = client.get_packets(Some(Duration::from_millis(100))) {
///             println!("{:#?}", packet);
///         }
///     }
/// }
pub struct FIRMClient {
    packet_receiver: Receiver<ProcessedFIRMData>,
    response_receiver: Receiver<FIRMResponse>,
    error_receiver: Receiver<String>,
    running: Arc<AtomicBool>,
    join_handle: Option<JoinHandle<Box<dyn SerialPort>>>,
    sender: Sender<ProcessedFIRMData>,
    response_sender: Sender<FIRMResponse>,
    error_sender: Sender<String>,
    command_sender: Sender<FIRMCommandPacket>,
    command_receiver: Option<Receiver<FIRMCommandPacket>>,
    mock_sender: Sender<FIRMLogPacket>,
    mock_receiver: Option<Receiver<FIRMLogPacket>>,
    port: Option<Box<dyn SerialPort>>,

    response_buffer: VecDeque<FIRMResponse>,

    mock_stream_stop: Arc<AtomicBool>,
    mock_stream_handle: Option<JoinHandle<anyhow::Result<usize>>>,

    calibration_snoop: Arc<RwLock<Option<Sender<ProcessedFIRMData>>>>,
    calibration_handle: Option<JoinHandle<Option<MagnetometerCalibration>>>,
}

impl FIRMClient {
    /// Creates a new FIRMClient instance connected to the specified serial port.
    ///
    /// # Arguments
    ///
    /// - `port_name` (`&str`) - The name of the serial port to connect to (e.g., "/dev/ttyUSB0").
    /// - `baud_rate` (`u32`) - The baud rate for the serial connection. Commonly 2,000,000 for FIRM devices.
    /// - `timeout` (`f64`) - Read timeout in seconds for the serial port.
    pub fn new(port_name: &str, baud_rate: u32, timeout: f64) -> Result<Self> {
        // Sets up the serial port
        let mut port: Box<dyn SerialPort> = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_millis((timeout * 1000.0) as u64))
            .open()
            .map_err(io::Error::other)?;

        // Sets DTR to true, this is important for Linux/Windows to both act the same
        port.write_data_terminal_ready(true)?;
        // Give the device a moment to settle after opening the port
        std::thread::sleep(Duration::from_millis(50));

        Ok(Self::new_from_port(port))
    }

    /// Creates a mocked client with a paired mock serial port and device handle.
    pub fn new_mock(timeout: f64) -> (Self, mock_serial::MockDeviceHandle) {
        let (port, device) = mock_serial::MockSerialPort::pair(Duration::from_secs_f64(timeout));
        let client = Self::new_from_port(port);
        (client, device)
    }

    fn new_from_port(port: Box<dyn SerialPort>) -> Self {
        let (sender, receiver) = channel();
        let (response_sender, response_receiver) = channel();
        let (error_sender, error_receiver) = channel();
        let (command_sender, command_receiver) = channel();
        let (mock_sender, mock_receiver) = channel();

        Self {
            packet_receiver: receiver,
            response_receiver,
            error_receiver,
            running: Arc::new(AtomicBool::new(false)),
            join_handle: None,
            sender,
            response_sender,
            error_sender,
            command_sender,
            command_receiver: Some(command_receiver),
            mock_sender,
            mock_receiver: Some(mock_receiver),
            port: Some(port),
            response_buffer: VecDeque::new(),

            mock_stream_stop: Arc::new(AtomicBool::new(false)),
            mock_stream_handle: None,

            calibration_snoop: Arc::new(RwLock::new(None)),
            calibration_handle: None,
        }
    }

    /// Starts the background thread to read from the serial port and parse packets.
    pub fn start(&mut self) {
        // Return early if already running
        if self.join_handle.is_some() {
            return;
        }

        // Gets the port or return if not available
        let mut port = match self.port.take() {
            Some(s) => s,
            None => return,
        };

        let command_receiver = match self.command_receiver.take() {
            Some(r) => r,
            None => return,
        };

        let mock_receiver = match self.mock_receiver.take() {
            Some(r) => r,
            None => return,
        };

        self.running.store(true, Ordering::Relaxed);
        // Clone variables for the thread. This way we can move them in, and the original ones
        // are still owned by self.
        let running_clone = self.running.clone();
        let sender = self.sender.clone();
        let response_sender = self.response_sender.clone();
        let error_sender = self.error_sender.clone();

        let calibration_snoop = self.calibration_snoop.clone();

        let handle: JoinHandle<Box<dyn SerialPort>> = thread::spawn(move || {
            let mut parser = SerialParser::new();
            // Buffer for reading from serial port
            let mut buffer: [u8; 1024] = [0; 1024];

            while running_clone.load(Ordering::Relaxed) {
                // Drain pending command packets first and write them to the port.
                while let Ok(cmd) = command_receiver.try_recv() {
                    let cmd_bytes = cmd.to_bytes();
                    // let hex = cmd_bytes
                    //     .iter()
                    //     .map(|b| format!("{:02X}", b))
                    //     .collect::<Vec<_>>()
                    //     .join(" ");
                    // println!("Command packet bytes: {hex}");
                    if let Err(e) = port.write_all(&cmd_bytes) {
                        let _ = error_sender.send(e.to_string());
                        running_clone.store(false, Ordering::Relaxed);
                        return port;
                    }
                }
                let _ = port.flush();

                // Then drain pending mock packets and write them to the port.
                while let Ok(packet) = mock_receiver.try_recv() {
                    let packet_bytes = packet.to_bytes();
                    // let hex = packet_bytes
                    //     .iter()
                    //     .map(|b| format!("{:02X}", b))
                    //     .collect::<Vec<_>>()
                    //     .join(" ");
                    // println!("Mock packet bytes: {hex}");

                    if let Err(e) = port.write_all(&packet_bytes) {
                        let _ = error_sender.send(e.to_string());
                        running_clone.store(false, Ordering::Relaxed);
                        return port;
                    }
                }
                let _ = port.flush();

                // Read bytes from the serial port
                match port.read(&mut buffer) {
                    Ok(bytes_read @ 1..) => {
                        // Feed the read bytes into the parser
                        parser.parse_bytes(&buffer[..bytes_read]);

                        // Reads all available data packets and send them to the main thread and calibration if wanted
                        while let Some(packet) = parser.get_data_packet() {
                            if sender.send(packet.clone()).is_err() {
                                return port; // Receiver dropped
                            }

                            // We use a read lock which is very fast if no one is writing.
                            if let Ok(guard) = calibration_snoop.read()
                                && let Some(cal_tx) = &*guard
                            {
                                // Ignore errors (if cal thread died, we don't care)
                                let _ = cal_tx.send(packet);
                            }
                        }

                        // Reads all available response packets and send them to the main thread
                        while let Some(firm_response_packet) = parser.get_response_packet() {
                            let response = firm_response_packet.response().clone();
                            if response_sender.send(response).is_err() {
                                return port; // Receiver dropped
                            }
                        }
                    }
                    Ok(0) => {}
                    // Timeouts might happen; just continue reading
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                    // Other errors should be reported and stop the thread:
                    Err(e) => {
                        let _ = error_sender.send(e.to_string());
                        running_clone.store(false, Ordering::Relaxed);
                        break;
                    }
                }
            }
            port
        });

        self.join_handle = Some(handle);
    }

    /// Stops the background thread and closes the serial port.
    pub fn stop(&mut self) {
        if let Err(e) = self.stop_mock_log_stream(false, true) {
            let _ = self.error_sender.send(e.to_string());
        }

        if self.calibration_handle.is_some() {
            let _ = self.finish_magnetometer_calibration();
        }

        self.running.store(false, Ordering::Relaxed);
        // todo: explain this properly when I understand it better (it's mostly for restarting)
        if let Some(handle) = self.join_handle.take()
            && let Ok(port) = handle.join()
        {
            self.port = Some(port);
        }

        // The receivers are moved into the background thread on start()
        // This remakes them so the client can be restarted.
        if self.command_receiver.is_none() {
            let (new_sender, new_receiver) = channel();
            self.command_sender = new_sender;
            self.command_receiver = Some(new_receiver);
        }

        if self.mock_receiver.is_none() {
            let (new_sender, new_receiver) = channel();
            self.mock_sender = new_sender;
            self.mock_receiver = Some(new_receiver);
        }
    }

    /// Retrieves all available data packets, optionally blocking until at least one is available.
    ///
    /// # Arguments
    ///
    /// - `timeout` (`Option<Duration>`) - If `Some(duration)`, the method will block for up to `duration` waiting for a packet.
    pub fn get_data_packets(
        &mut self,
        timeout: Option<Duration>,
    ) -> Result<Vec<ProcessedFIRMData>, RecvTimeoutError> {
        let mut packets = Vec::new();

        // If blocking, wait for at most one packet. The next loop will drain any others.
        if let Some(duration) = timeout {
            packets.push(self.packet_receiver.recv_timeout(duration)?);
        }

        // Drains the rest of the available packets without blocking
        while let Ok(packet) = self.packet_receiver.try_recv() {
            packets.push(packet);
        }
        Ok(packets)
    }

    /// Retrieves all available response packets, optionally blocking until at least one is available.
    ///
    /// # Arguments
    ///
    /// - `timeout` (`Option<Duration>`) - If `Some(duration)`, the method will block for up to `duration` waiting for a response.
    pub fn get_response_packets(
        &mut self,
        timeout: Option<Duration>,
    ) -> Result<Vec<FIRMResponse>, RecvTimeoutError> {
        let mut responses: Vec<FIRMResponse> = self.response_buffer.drain(..).collect();

        // If blocking and we have nothing buffered, wait for one response.
        if responses.is_empty()
            && let Some(duration) = timeout
        {
            responses.push(self.response_receiver.recv_timeout(duration)?);
        }

        while let Ok(res) = self.response_receiver.try_recv() {
            responses.push(res);
        }

        Ok(responses)
    }

    /// Requests device info and waits for the response.
    pub fn get_device_info(&mut self, timeout: Duration) -> Result<Option<DeviceInfo>> {
        self.send_command(FIRMCommandPacket::build_get_device_info_command())?;
        self.wait_for_matching_response(timeout, |res| match res {
            FIRMResponse::GetDeviceInfo(info) => Some(info.clone()),
            _ => None,
        })
    }

    /// Requests device configuration and waits for the response.
    pub fn get_device_config(&mut self, timeout: Duration) -> Result<Option<DeviceConfig>> {
        self.send_command(FIRMCommandPacket::build_get_device_config_command())?;
        self.wait_for_matching_response(timeout, |res| match res {
            FIRMResponse::GetDeviceConfig(cfg) => Some(cfg.clone()),
            _ => None,
        })
    }

    /// Sets device configuration and waits for acknowledgement.
    pub fn set_device_config(
        &mut self,
        name: String,
        frequency: u16,
        protocol: DeviceProtocol,
        timeout: Duration,
    ) -> Result<Option<bool>> {
        let config = DeviceConfig {
            name,
            frequency,
            protocol,
        };
        self.send_command(FIRMCommandPacket::build_set_device_config_command(config))?;
        self.wait_for_matching_response(timeout, |res| match res {
            FIRMResponse::SetDeviceConfig(ok) => Some(*ok),
            _ => None,
        })
    }

    pub fn set_magnetometer_calibration(
        &mut self,
        offsets: [f32; NUMBER_OF_CALIBRATION_OFFSETS],
        scale_matrix: [f32; NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS],
        timeout: Duration,
    ) -> Result<Option<bool>> {
        self.send_command(
            FIRMCommandPacket::build_set_magnetometer_calibration_command(offsets, scale_matrix),
        )?;
        self.wait_for_matching_response(timeout, |res| match res {
            FIRMResponse::SetMagnetometerCalibration(ok) => Some(*ok),
            _ => None,
        })
    }

    pub fn set_imu_calibration(
        &mut self,
        accel_offsets: [f32; NUMBER_OF_CALIBRATION_OFFSETS],
        accel_scale_matrix: [f32; NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS],
        gyro_offsets: [f32; NUMBER_OF_CALIBRATION_OFFSETS],
        gyro_scale_matrix: [f32; NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS],
        timeout: Duration,
    ) -> Result<Option<bool>> {
        self.send_command(FIRMCommandPacket::build_set_imu_calibration_command(
            accel_offsets,
            accel_scale_matrix,
            gyro_offsets,
            gyro_scale_matrix,
        ))?;
        self.wait_for_matching_response(timeout, |res| match res {
            FIRMResponse::SetIMUCalibration(ok) => Some(*ok),
            _ => None,
        })
    }

    pub fn get_calibration(&mut self, timeout: Duration) -> Result<Option<CalibrationValues>> {
        self.send_command(FIRMCommandPacket::build_get_calibration_command())?;
        self.wait_for_matching_response(timeout, |res| match res {
            FIRMResponse::GetCalibration(calibration) => Some(calibration.clone()),
            _ => None,
        })
    }

    /// Starts streaming a `.frm` mock log file on a background thread.
    ///
    /// While the mock stream is running you can continue to call `get_data_packets()` or other
    /// APIs to read/log FIRM's output.
    ///
    /// Use `is_mock_log_streaming()` to check status and `try_join_mock_log_stream()` to
    /// retrieve the sent packet count when finished.
    pub fn start_mock_log_stream(
        &mut self,
        log_path: String,
        start_timeout: Duration,
        realtime: bool,
        speed: f64,
        chunk_size: usize,
        cancel_on_finish: bool,
    ) -> Result<()> {
        if self.is_mock_log_streaming() {
            return Err(anyhow::anyhow!("Mock stream already running"));
        }

        if !self.is_running() {
            self.start();
        }

        // If a previous stream finished but wasn't joined, join it now.
        if self
            .mock_stream_handle
            .as_ref()
            .is_some_and(|h| h.is_finished())
        {
            let _ = self.stop_mock_log_stream(false, true);
        }

        self.start_mock_mode(start_timeout)?;

        self.mock_stream_stop.store(false, Ordering::Relaxed);
        let stop = self.mock_stream_stop.clone();
        let mock_sender = self.mock_sender.clone();
        let command_sender = self.command_sender.clone();
        let error_sender = self.error_sender.clone();

        let handle = thread::spawn(move || {
            let result = stream_mock_log_file_worker(
                &log_path,
                realtime,
                speed,
                chunk_size,
                &stop,
                &mock_sender,
            );

            if cancel_on_finish {
                // Fire-and-forget: we can't wait for ack from this background thread.
                let _ = command_sender.send(FIRMCommandPacket::build_cancel_command());
            }

            if let Err(ref e) = result {
                let _ = error_sender.send(e.to_string());
            }
            result
        });

        self.mock_stream_handle = Some(handle);
        Ok(())
    }

    /// Returns `true` if a background mock stream is currently running.
    pub fn is_mock_log_streaming(&self) -> bool {
        self.mock_stream_handle
            .as_ref()
            .is_some_and(|h| !h.is_finished())
    }

    pub fn stop_mock_log_stream(
        &mut self,
        cancel_device: bool,
        block: bool,
    ) -> Result<Option<usize>> {
        self.mock_stream_stop.store(true, Ordering::Relaxed);

        if cancel_device {
            let _ = self
                .command_sender
                .send(FIRMCommandPacket::build_cancel_command());
        }

        if !block {
            // non-blocking: only join if already finished
            let Some(h) = self.mock_stream_handle.as_ref() else {
                return Ok(None);
            };
            if !h.is_finished() {
                return Ok(None);
            }
        }

        let Some(handle) = self.mock_stream_handle.take() else {
            return Ok(None);
        };

        let res = handle
            .join()
            .map_err(|_| anyhow::anyhow!("Mock stream thread panicked"))??;

        Ok(Some(res))
    }

    /// Sends a cancel command and waits for acknowledgement.
    pub fn cancel(&mut self, timeout: Duration) -> Result<Option<bool>> {
        self.send_command(FIRMCommandPacket::build_cancel_command())?;
        self.wait_for_matching_response(timeout, |res| match res {
            FIRMResponse::Cancel(ok) => Some(*ok),
            _ => None,
        })
    }

    /// Sends a reboot command.
    pub fn reboot(&self) -> Result<()> {
        self.send_command(FIRMCommandPacket::build_reboot_command())
    }

    /// Checks for any errors that have occurred in the background thread.
    ///
    /// # Returns
    ///
    /// - `Option<String>` - `Some(error_message)` if an error has occurred, otherwise `None`.
    pub fn check_error(&self) -> Option<String> {
        self.error_receiver.try_recv().ok()
    }

    /// Returns true if the client is currently running and reading data.
    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }

    /// Enter mock mode and require an acknowledgement.
    ///
    /// Returns `Ok(())` only if the device explicitly acknowledges mock mode.
    fn start_mock_mode(&mut self, timeout: Duration) -> Result<()> {
        self.send_command(FIRMCommandPacket::build_mock_command())?;

        match self.wait_for_matching_response(timeout, |res| match res {
            FIRMResponse::Mock(ok) => Some(*ok),
            _ => None,
        })? {
            Some(true) => Ok(()),
            Some(false) => Err(anyhow::anyhow!("Device rejected mock mode")),
            None => Err(anyhow::anyhow!(
                "Timed out waiting for mock acknowledgement"
            )),
        }
    }

    /// Runs a full magnetometer calibration sequence for a specific duration and applies the result.
    ///
    /// This is a blocking helper function that:
    /// 1. Starts the background calibration listener.
    /// 2. Sleeps for `collection_duration` (allows you to rotate the device).
    /// 3. Stops the listener and calculates the offsets/matrix.
    /// 4. Automatically sends the new calibration to the device.
    ///
    /// # Returns
    /// - `Ok(Some(true))` if the calibration was calculated and accepted by the device.
    /// - `Ok(None)` if the calibration failed (not enough data) or device did not ack.
    /// - `Err(...)` if there was a communication error.
    pub fn run_and_apply_magnetometer_calibration(
        &mut self,
        collection_duration: Duration,
        apply_timeout: Duration,
    ) -> Result<Option<bool>> {
        // Reset magnetometer calibration to a known state before collecting.
        // This avoids using stale calibration while we gather new samples.
        let zero_offsets: [f32; NUMBER_OF_CALIBRATION_OFFSETS] =
            [0.0; NUMBER_OF_CALIBRATION_OFFSETS];
        let identity_matrix: [f32; NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS] =
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];

        match self.set_magnetometer_calibration(zero_offsets, identity_matrix, apply_timeout)? {
            Some(true) => {}
            _ => return Ok(None),
        }

        // 1. Start listening
        self.start_magnetometer_calibration()?;

        // 2. Wait for data collection (Block thread while user rotates device)
        // We use std::thread::sleep because this function is intended to be blocking.
        std::thread::sleep(collection_duration);

        // 3. Finish and Calculate
        let calibration_result = self.finish_magnetometer_calibration()?;

        match calibration_result {
            Some((offsets, matrix)) => {
                // 4. Apply to device
                // We have valid data, so send the set command.
                println!(
                    "Calibration result:\\n  b = [{:.6}, {:.6}, {:.6}]",
                    offsets[0], offsets[1], offsets[2]
                );

                let a = [
                    matrix[0], matrix[3], matrix[6], matrix[1], matrix[4], matrix[7], matrix[2],
                    matrix[5], matrix[8],
                ];

                println!(
                    "  A = [{:.6}, {:.6}, {:.6}; {:.6}, {:.6}, {:.6}; {:.6}, {:.6}, {:.6}]",
                    a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8]
                );

                self.set_magnetometer_calibration(offsets, matrix, apply_timeout)
            }
            None => {
                // Calibration failed (likely not enough distinct data points)
                Ok(None)
            }
        }
    }

    /// Sends a high-level command to the device.
    fn send_command(&self, command: FIRMCommandPacket) -> Result<()> {
        self.command_sender
            .send(command)
            .map_err(|_| io::Error::other("Command channel closed"))?;
        Ok(())
    }

    fn wait_for_response(&mut self, timeout: Duration) -> Result<Option<FIRMResponse>> {
        // Prefer already-buffered responses.
        if let Some(res) = self.response_buffer.pop_front() {
            return Ok(Some(res));
        }

        match self.response_receiver.recv_timeout(timeout) {
            Ok(res) => Ok(Some(res)),
            Err(RecvTimeoutError::Timeout) => Ok(None),
            Err(RecvTimeoutError::Disconnected) => {
                Err(io::Error::other("Response channel closed").into())
            }
        }
    }

    /// Wait for a response matching `matcher` up to `timeout`.
    ///
    /// Looks through buffered responses first and keeps non-matching responses. It makes sure
    /// that the total time spent waiting is less than `timeout`.
    fn wait_for_matching_response<T>(
        &mut self,
        timeout: Duration,
        mut matcher: impl FnMut(&FIRMResponse) -> Option<T>,
    ) -> Result<Option<T>> {
        // Pull any immediately-available responses into our buffer so we can search them first.
        while let Ok(res) = self.response_receiver.try_recv() {
            self.response_buffer.push_back(res);
        }

        let mut try_get_response = |response_buffer: &mut VecDeque<FIRMResponse>| {
            // First, search the buffer for a match without blocking.
            if let Some((idx, value)) = response_buffer
                .iter()
                .enumerate()
                .find_map(|(idx, res)| matcher(res).map(|value| (idx, value)))
            {
                // Remove the matched response from the buffer and return it.
                response_buffer.remove(idx);
                Some(value)
            } else {
                None
            }
        };

        if let Some(result) = try_get_response(&mut self.response_buffer) {
            return Ok(Some(result));
        }

        // Makes a deadline to enforce the overall timeout
        let deadline = std::time::Instant::now() + timeout;

        loop {
            // If we've already passed the deadline, give up and return None.
            let now = std::time::Instant::now();
            if now >= deadline {
                return Ok(None);
            }

            let remaining = deadline - now;

            // If the receiver was disconnected, propagate the error.
            let Some(next) = self.wait_for_response(remaining)? else {
                // No response arrived before the remaining time elapsed.
                return Ok(None);
            };

            // Keep the response in the buffer so non-matching responses are kept for other calls
            self.response_buffer.push_back(next);

            // Re-scan the buffer for a match now that we have new data.
            if let Some(result) = try_get_response(&mut self.response_buffer) {
                return Ok(Some(result));
            }
        }
    }

    /// Starts the magnetometer calibration process in a background thread.
    ///
    /// This will automatically begin collecting data from the incoming stream.
    /// Call `finish_magnetometer_calibration` to stop collecting and calculate the result.
    pub fn start_magnetometer_calibration(&mut self) -> Result<()> {
        if self.calibration_handle.is_some() {
            return Err(anyhow::anyhow!("Calibration already in progress"));
        }

        // Create a channel for the serial thread to send data to the calibration thread
        let (tx, rx) = channel();

        // Register the sender in the shared snoop slot.
        // The serial thread will pick this up on its next loop iteration.
        {
            let mut guard = self
                .calibration_snoop
                .write()
                .map_err(|_| anyhow::anyhow!("Lock poisoned"))?;
            *guard = Some(tx);
        }

        // Spawn the calibration thread
        let handle = thread::spawn(move || {
            let mut calibrator = MagnetometerCalibrator::new();
            calibrator.start();

            // Keep receiving data until the sender is dropped (which happens in finish_calibration)
            while let Ok(data) = rx.recv() {
                calibrator.add_sample(&data);
            }

            calibrator.stop();
            calibrator.calculate()
        });

        self.calibration_handle = Some(handle);
        Ok(())
    }

    /// Stops the calibration process and calculates the hard iron offsets and soft iron matrix.
    ///
    /// Returns `Ok(None)` if the calibration failed (e.g. not enough data points).
    pub fn finish_magnetometer_calibration(&mut self) -> Result<Option<([f32; 3], [f32; 9])>> {
        // 1. Remove the sender from the snoop slot.
        // This causes the `rx.recv()` in the calibration thread to return an error, breaking its loop.
        {
            let mut guard = self
                .calibration_snoop
                .write()
                .map_err(|_| anyhow::anyhow!("Lock poisoned"))?;
            *guard = None;
        }

        // 2. Join the thread to get the result
        if let Some(handle) = self.calibration_handle.take() {
            let result = handle
                .join()
                .map_err(|_| anyhow::anyhow!("Calibration thread panicked"))?;

            // 3. Convert result to the array format we need for FIRMCommand
            if let Some(cal) = result {
                return Ok(Some(cal.to_arrays()));
            }
        }

        Ok(None)
    }
}

fn sleep_interruptible(total: Duration, stop: &AtomicBool) {
    let step = Duration::from_millis(10);
    let mut remaining = total;
    while remaining > Duration::ZERO && !stop.load(Ordering::Relaxed) {
        let s = remaining.min(step);
        thread::sleep(s);
        remaining = remaining.saturating_sub(s);
    }
}

fn stream_mock_log_file_worker(
    log_path: &str,
    realtime: bool,
    speed: f64,
    chunk_size: usize,
    stop: &AtomicBool,
    mock_sender: &Sender<FIRMLogPacket>,
) -> Result<usize> {
    if speed <= 0.0 {
        return Err(anyhow::anyhow!("speed must be > 0"));
    }

    const PRELOAD_COUNT: usize = 75;
    const BATCH_SIZE: usize = 10;

    let mut file = File::open(log_path)?;
    let mut header = vec![0u8; HEADER_TOTAL_SIZE];
    file.read_exact(&mut header)?;

    // Send the log header to the device, framed as a mock packet.
    let header_packet = FIRMLogPacket::new(FIRMLogPacketType::HeaderPacket, header.clone());
    mock_sender
        .send(header_packet)
        .map_err(|_| io::Error::other("Mock channel closed"))?;

    let mut parser = LogParser::new();
    parser.read_header(&header);

    // After we send the header we pause for a short time to let the device process it.
    sleep_interruptible(HEADER_PARSE_DELAY, stop);

    let mut buf = vec![0u8; chunk_size];
    let mut packets_sent = 0usize;

    // Staging queue of parsed packets + their requested delay.
    let mut staged: std::collections::VecDeque<(FIRMLogPacket, f64)> =
        std::collections::VecDeque::new();

    // Read+parse enough bytes to stage more packets.
    // Returns Ok(true) if it read more bytes, Ok(false) if file is exhausted.
    let refill = |file: &mut File,
                  buf: &mut [u8],
                  parser: &mut LogParser,
                  staged: &mut std::collections::VecDeque<(FIRMLogPacket, f64)>|
     -> Result<bool> {
        let n = file.read(buf)?;
        if n == 0 {
            return Ok(false);
        }

        parser.parse_bytes(&buf[..n]);
        while let Some((packet, delay_seconds)) = parser.get_packet_and_time_delay() {
            staged.push_back((packet, delay_seconds));
        }

        Ok(true)
    };

    // -------------------------
    // 1) PRELOAD: send 75 ASAP
    // -------------------------
    let mut sent_preload = 0usize;
    while sent_preload < PRELOAD_COUNT && !stop.load(Ordering::Relaxed) {
        // Ensure we have at least one staged packet.
        while staged.is_empty() {
            let read_more = refill(&mut file, &mut buf, &mut parser, &mut staged)?;
            if !read_more {
                // File ended; drain anything remaining in parser.
                while let Some((packet, delay_seconds)) = parser.get_packet_and_time_delay() {
                    staged.push_back((packet, delay_seconds));
                }
                break;
            }
        }

        let Some((packet, _delay_seconds)) = staged.pop_front() else {
            break; // no more packets available
        };

        // Send immediately (no pacing / no sleeping).
        mock_sender
            .send(packet)
            .map_err(|_| io::Error::other("Mock channel closed"))?;
        packets_sent += 1;
        sent_preload += 1;
    }

    // After the burst, reset pacing so realtime timing starts “from here”.
    let stream_start = Instant::now();
    let mut total_delay_seconds = 0.0f64;

    // -----------------------------------------
    // 2) MAIN: send in batches of 10 packets
    // -----------------------------------------
    loop {
        if stop.load(Ordering::Relaxed) {
            break;
        }

        // Make sure we can form a batch (or hit EOF).
        while staged.len() < BATCH_SIZE {
            let read_more = refill(&mut file, &mut buf, &mut parser, &mut staged)?;
            if !read_more {
                // File ended; drain any remaining packets parser can produce.
                while let Some((packet, delay_seconds)) = parser.get_packet_and_time_delay() {
                    staged.push_back((packet, delay_seconds));
                }
                break;
            }
        }

        if staged.is_empty() {
            break;
        }

        // Pop up to BATCH_SIZE packets.
        let mut batch_delay = 0.0f64;
        let mut batch = Vec::with_capacity(BATCH_SIZE);
        for _ in 0..BATCH_SIZE {
            if let Some((packet, delay_seconds)) = staged.pop_front() {
                batch_delay += delay_seconds;
                batch.push(packet);
            } else {
                break;
            }
        }

        // Send the whole batch (still individual send() calls, but no per-packet sleep).
        for packet in batch {
            if stop.load(Ordering::Relaxed) {
                break;
            }
            mock_sender
                .send(packet)
                .map_err(|_| io::Error::other("Mock channel closed"))?;
            packets_sent += 1;
        }

        // Sleep once per batch to approximate original pacing.
        if realtime && batch_delay > 0.0 {
            total_delay_seconds += batch_delay;

            let stream_elapsed = stream_start.elapsed().as_secs_f64();
            if stream_elapsed <= total_delay_seconds / speed {
                sleep_interruptible(Duration::from_secs_f64(batch_delay / speed), stop);
            }
        }

        // If parser knows it hit EOF and nothing is staged, we’re done.
        if parser.eof_reached() && staged.is_empty() {
            break;
        }
    }

    Ok(packets_sent)
}

/// Ensures that the client is properly stopped when dropped, i.e. .stop() is called.
impl Drop for FIRMClient {
    fn drop(&mut self) {
        self.stop();
    }
}

#[cfg(test)]
mod tests {
    use firm_core::{
        constants::{
            command::{
                DEVICE_ID_LENGTH, DEVICE_NAME_LENGTH, FIRMCommand, FIRMWARE_VERSION_LENGTH,
                FREQUENCY_LENGTH,
            },
            packet::PacketHeader,
        },
        firm_packets::FIRMResponsePacket,
        framed_packet::FramedPacket,
    };

    use super::*;

    fn str_to_bytes<const N: usize>(string: &str) -> [u8; N] {
        let mut out = [0u8; N];
        let bytes = string.as_bytes();
        let n = bytes.len().min(N);
        out[..n].copy_from_slice(&bytes[..n]);
        out
    }

    #[test]
    fn test_new_failure() {
        // Test that creating a client with an invalid port fails immediately
        let result = FIRMClient::new("invalid_port_name", 2_000_000, 0.1);
        assert!(result.is_err());
    }

    #[test]
    fn test_start_stop() {
        let (mut client, _device) = FIRMClient::new_mock(0.01);

        assert!(!client.is_running());
        client.start();
        assert!(client.is_running());
        client.stop();
        assert!(!client.is_running());
    }

    #[test]
    fn test_get_data_packet_over_mock_serial() {
        let (mut client, device) = FIRMClient::new_mock(0.01);
        client.start();

        let timestamp_seconds = 1.5f64;

        let mut payload = vec![0u8; 120];
        payload[0..8].copy_from_slice(&timestamp_seconds.to_le_bytes());
        payload[8..12].copy_from_slice(&25.0f32.to_le_bytes());

        let mocked_packet = FramedPacket::new(PacketHeader::Data, 0, payload);
        device.inject_framed_packet(mocked_packet);

        // Need to give some time for the background thread to read the data
        let packets = client
            .get_data_packets(Some(Duration::from_millis(100)))
            .unwrap();
        assert!(!packets.is_empty());
        assert!((packets[0].timestamp_seconds - timestamp_seconds).abs() < 1e-9);
    }

    #[test]
    fn test_get_response_packet_over_mock_serial() {
        let (mut client, device) = FIRMClient::new_mock(0.01);
        client.start();

        let payload = [1u8];

        let bytes = FramedPacket::new(
            PacketHeader::Response,
            FIRMCommand::SetDeviceConfig.to_u16(),
            payload.to_vec(),
        )
        .to_bytes();
        let response_packet = FIRMResponsePacket::from_bytes(&bytes).unwrap();

        device.inject_framed_packet(response_packet.frame().clone());

        let packet = client
            .get_response_packets(Some(Duration::from_millis(100)))
            .unwrap();

        // Make sure we didn't get any other type of packets
        assert!(matches!(
            client.get_data_packets(Some(Duration::from_millis(10))),
            Err(RecvTimeoutError::Timeout)
        ));

        // Make sure we got the expected response
        assert_eq!(packet.len(), payload.len());
        assert_eq!(packet[0], FIRMResponse::SetDeviceConfig(true));

        // Make sure we didn't get any extra response packets
        assert!(matches!(
            client.get_response_packets(Some(Duration::from_millis(10))),
            Err(RecvTimeoutError::Timeout)
        ));
    }

    #[test]
    fn test_set_device_config_command() {
        let (mut client, device) = FIRMClient::new_mock(0.01);
        client.start();

        // Prepare the response packet to be injected
        let response_payload = [1u8]; // Acknowledgement byte
        let response_packet = FramedPacket::new(
            PacketHeader::Response,
            FIRMCommand::SetDeviceConfig.to_u16(),
            response_payload.to_vec(),
        );
        device.inject_framed_packet(response_packet);

        // Send the set device config command
        let result = client.set_device_config(
            "TestDevice".to_string(),
            100,
            DeviceProtocol::UART,
            Duration::from_millis(100),
        );

        // Verify the result
        assert_eq!(result.unwrap(), Some(true));
    }

    #[test]
    fn test_get_device_info_command() {
        let (mut client, device) = FIRMClient::new_mock(0.01);
        client.start();

        let id = 0x1122334455667788u64;
        let mut payload = vec![0u8; DEVICE_ID_LENGTH + FIRMWARE_VERSION_LENGTH];
        payload[0..DEVICE_ID_LENGTH].copy_from_slice(&id.to_le_bytes());
        let fw_bytes = str_to_bytes::<FIRMWARE_VERSION_LENGTH>("v1.2.3");
        payload[DEVICE_ID_LENGTH..DEVICE_ID_LENGTH + FIRMWARE_VERSION_LENGTH]
            .copy_from_slice(&fw_bytes);

        let response_packet = FramedPacket::new(
            PacketHeader::Response,
            FIRMCommand::GetDeviceInfo.to_u16(),
            payload,
        );
        device.inject_framed_packet(response_packet);

        let result = client.get_device_info(Duration::from_millis(100));

        assert_eq!(
            result.unwrap(),
            Some(DeviceInfo {
                firmware_version: "v1.2.3".to_string(),
                id,
            })
        );
    }

    #[test]
    fn test_get_device_config_command() {
        let (mut client, device) = FIRMClient::new_mock(0.01);
        client.start();

        let name = "TestDevice";
        let frequency: u16 = 100;
        let protocol = DeviceProtocol::UART;

        let mut payload = vec![0u8; DEVICE_NAME_LENGTH + FREQUENCY_LENGTH + 1];
        let name_bytes = str_to_bytes::<DEVICE_NAME_LENGTH>(name);
        payload[0..DEVICE_NAME_LENGTH].copy_from_slice(&name_bytes);
        payload[DEVICE_NAME_LENGTH..DEVICE_NAME_LENGTH + FREQUENCY_LENGTH]
            .copy_from_slice(&frequency.to_le_bytes());
        payload[DEVICE_NAME_LENGTH + FREQUENCY_LENGTH] = 2;

        let response_packet = FramedPacket::new(
            PacketHeader::Response,
            FIRMCommand::GetDeviceConfig.to_u16(),
            payload,
        );
        device.inject_framed_packet(response_packet);

        let result = client.get_device_config(Duration::from_millis(100));

        assert_eq!(
            result.unwrap(),
            Some(DeviceConfig {
                name: name.to_string(),
                frequency,
                protocol,
            })
        );
    }

    #[test]
    fn test_get_calibration_command() {
        let (mut client, device) = FIRMClient::new_mock(0.01);
        client.start();

        let expected = CalibrationValues {
            imu_accelerometer_offsets: [1.0, 2.0, 3.0],
            imu_accelerometer_scale_matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            imu_gyroscope_offsets: [4.0, 5.0, 6.0],
            imu_gyroscope_scale_matrix: [2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0],
            magnetometer_offsets: [7.0, 8.0, 9.0],
            magnetometer_scale_matrix: [3.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0],
        };

        let mut payload: Vec<u8> = Vec::new();
        for v in expected.imu_accelerometer_offsets {
            payload.extend_from_slice(&v.to_le_bytes());
        }
        for v in expected.imu_accelerometer_scale_matrix {
            payload.extend_from_slice(&v.to_le_bytes());
        }
        for v in expected.imu_gyroscope_offsets {
            payload.extend_from_slice(&v.to_le_bytes());
        }
        for v in expected.imu_gyroscope_scale_matrix {
            payload.extend_from_slice(&v.to_le_bytes());
        }
        for v in expected.magnetometer_offsets {
            payload.extend_from_slice(&v.to_le_bytes());
        }
        for v in expected.magnetometer_scale_matrix {
            payload.extend_from_slice(&v.to_le_bytes());
        }

        let response_packet = FramedPacket::new(
            PacketHeader::Response,
            FIRMCommand::GetCalibration.to_u16(),
            payload,
        );
        device.inject_framed_packet(response_packet);

        let result = client.get_calibration(Duration::from_millis(100));

        assert_eq!(result.unwrap(), Some(expected));
    }

    #[test]
    fn test_cancel_command() {
        let (mut client, device) = FIRMClient::new_mock(0.01);
        client.start();

        let response_payload = [1u8];
        let response_packet = FramedPacket::new(
            PacketHeader::Response,
            FIRMCommand::Cancel.to_u16(),
            response_payload.to_vec(),
        );
        device.inject_framed_packet(response_packet);

        let result = client.cancel(Duration::from_millis(100));

        assert_eq!(result.unwrap(), Some(true));
    }
}
