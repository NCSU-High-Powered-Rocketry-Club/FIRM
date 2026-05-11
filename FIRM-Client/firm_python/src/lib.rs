use firm_core::constants::command::{
    NUMBER_OF_CALIBRATION_OFFSETS, NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS,
};
use firm_core::constants::packet::PacketHeader;
use firm_core::firm_packets::{
    CalibrationValues, DeviceConfig, DeviceInfo, DeviceProtocol, ProcessedFIRMData,
};
use firm_core::framed_packet::FramedPacket;
use firm_rust::FIRMClient as RustFirmClient;
use firm_rust::mock_serial::MockDeviceHandle as RustMockDeviceHandle;
use pyo3::prelude::*;
use std::time::Duration;

#[inline]
fn py_io_err(msg: impl ToString) -> PyErr {
    pyo3::exceptions::PyIOError::new_err(msg.to_string())
}

#[inline]
fn map_io<T>(res: Result<T, impl ToString>) -> PyResult<T> {
    res.map_err(py_io_err)
}

#[pyclass(unsendable)]
struct FIRMClient {
    inner: RustFirmClient,
    /// Used only when `get_data_packets(block=true)` is called.
    timeout: f64,
}

#[pyclass(unsendable)]
struct MockDeviceHandle {
    inner: RustMockDeviceHandle,
}

#[pymethods]
impl FIRMClient {
    #[new]
    #[pyo3(signature = (port_name, baud_rate=2_000_000, timeout=0.1))]
    fn new(port_name: &str, baud_rate: Option<u32>, timeout: Option<f64>) -> PyResult<Self> {
        let baudrate = baud_rate.unwrap_or(2_000_000);
        let timeout_val = timeout.unwrap_or(0.1);

        // Opens the client and gives descriptive error messages on failure
        let client = RustFirmClient::new(port_name, baudrate, timeout_val).map_err(|e| {
            if let Some(io_err) = e.downcast_ref::<std::io::Error>() {
                use std::io::ErrorKind;

                match io_err.kind() {
                    ErrorKind::NotFound => py_io_err(format!(
                        "Serial port '{}' not found. \
                        Check the port name (e.g. COM8, /dev/ttyACM0).",
                        port_name
                    )),
                    ErrorKind::PermissionDenied => py_io_err(format!(
                        "Permission denied opening serial port '{}'. \
                        Try running as admin or fixing udev permissions.",
                        port_name
                    )),
                    _ => py_io_err(format!(
                        "Failed to open serial port '{}': {}",
                        port_name, io_err
                    )),
                }
            } else {
                // Non-IO error (logic error, config error, etc.)
                py_io_err(format!(
                    "Failed to initialize FIRM client for '{}': {}",
                    port_name, e
                ))
            }
        })?;

        Ok(Self {
            inner: client,
            timeout: timeout_val,
        })
    }

    #[staticmethod]
    #[pyo3(signature = (timeout=0.1))]
    fn new_mock(timeout: f64) -> PyResult<(Self, MockDeviceHandle)> {
        let (client, device) = RustFirmClient::new_mock(timeout);
        Ok((
            Self {
                inner: client,
                timeout,
            },
            MockDeviceHandle { inner: device },
        ))
    }

    #[inline]
    fn ensure_ok(&self) -> PyResult<()> {
        if let Some(err) = self.inner.check_error() {
            return Err(py_io_err(err));
        }
        Ok(())
    }

    fn start(&mut self) -> PyResult<()> {
        self.inner.start();
        Ok(())
    }

    fn stop(&mut self) {
        self.inner.stop();
    }

    /// Start streaming a mock log file in the background.
    #[pyo3(signature = (log_path, realtime=true, speed=1.0, chunk_size=8192, start_timeout_seconds=5.0, cancel_on_finish=true))]
    fn start_mock_log_stream(
        &mut self,
        log_path: &str,
        realtime: bool,
        speed: f64,
        chunk_size: usize,
        start_timeout_seconds: f64,
        cancel_on_finish: bool,
    ) -> PyResult<()> {
        self.ensure_ok()?;

        map_io(self.inner.start_mock_log_stream(
            log_path.to_string(),
            Duration::from_secs_f64(start_timeout_seconds),
            realtime,
            speed,
            chunk_size,
            cancel_on_finish,
        ))?;

        Ok(())
    }

    fn is_mock_log_streaming(&self) -> bool {
        self.inner.is_mock_log_streaming()
    }

    #[pyo3(signature = (cancel_device=true, block=true))]
    fn stop_mock_log_stream(
        &mut self,
        cancel_device: bool,
        block: bool,
    ) -> PyResult<Option<usize>> {
        let res = map_io(self.inner.stop_mock_log_stream(cancel_device, block))?;
        Ok(res)
    }

    #[pyo3(signature = (block=false))]
    fn get_data_packets(&mut self, block: bool) -> PyResult<Vec<ProcessedFIRMData>> {
        self.ensure_ok()?;

        let timeout = if block {
            Some(Duration::from_secs_f64(self.timeout))
        } else {
            None
        };

        let packets = map_io(self.inner.get_data_packets(timeout))?;
        Ok(packets)
    }

    #[pyo3(signature = (timeout_seconds=5.0))]
    fn get_device_info(&mut self, timeout_seconds: f64) -> PyResult<Option<DeviceInfo>> {
        self.ensure_ok()?;
        let info = map_io(
            self.inner
                .get_device_info(Duration::from_secs_f64(timeout_seconds)),
        )?;
        Ok(info)
    }

    #[pyo3(signature = (timeout_seconds=5.0))]
    fn get_device_config(&mut self, timeout_seconds: f64) -> PyResult<Option<DeviceConfig>> {
        self.ensure_ok()?;
        let cfg = map_io(
            self.inner
                .get_device_config(Duration::from_secs_f64(timeout_seconds)),
        )?;
        Ok(cfg)
    }

    #[pyo3(signature = (name, frequency, protocol, timeout_seconds=5.0))]
    fn set_device_config(
        &mut self,
        name: String,
        frequency: u16,
        protocol: DeviceProtocol,
        timeout_seconds: f64,
    ) -> PyResult<bool> {
        self.ensure_ok()?;

        let res = map_io(self.inner.set_device_config(
            name,
            frequency,
            protocol,
            Duration::from_secs_f64(timeout_seconds),
        ))?;

        Ok(res.unwrap_or(false))
    }

    #[pyo3(signature = (offsets, scale_matrix, timeout_seconds=5.0))]
    fn set_magnetometer_calibration(
        &mut self,
        offsets: [f32; NUMBER_OF_CALIBRATION_OFFSETS],
        scale_matrix: [f32; NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS],
        timeout_seconds: f64,
    ) -> PyResult<bool> {
        self.ensure_ok()?;

        let res = map_io(self.inner.set_magnetometer_calibration(
            offsets,
            scale_matrix,
            Duration::from_secs_f64(timeout_seconds),
        ))?;

        Ok(res.unwrap_or(false))
    }

    #[pyo3(signature = (accel_offsets, accel_scale_matrix, gyro_offsets, gyro_scale_matrix, timeout_seconds=5.0))]
    fn set_imu_calibration(
        &mut self,
        accel_offsets: [f32; NUMBER_OF_CALIBRATION_OFFSETS],
        accel_scale_matrix: [f32; NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS],
        gyro_offsets: [f32; NUMBER_OF_CALIBRATION_OFFSETS],
        gyro_scale_matrix: [f32; NUMBER_OF_CALIBRATION_SCALE_MATRIX_ELEMENTS],
        timeout_seconds: f64,
    ) -> PyResult<bool> {
        self.ensure_ok()?;

        let res = map_io(self.inner.set_imu_calibration(
            accel_offsets,
            accel_scale_matrix,
            gyro_offsets,
            gyro_scale_matrix,
            Duration::from_secs_f64(timeout_seconds),
        ))?;

        Ok(res.unwrap_or(false))
    }

    #[pyo3(signature = (timeout_seconds=5.0))]
    fn get_calibration(&mut self, timeout_seconds: f64) -> PyResult<Option<CalibrationValues>> {
        self.ensure_ok()?;
        let calibration = map_io(
            self.inner
                .get_calibration(Duration::from_secs_f64(timeout_seconds)),
        )?;
        Ok(calibration)
    }

    /// Runs a blocking magnetometer calibration sequence.
    ///
    /// This function will:
    /// 1. Start the calibration listener.
    /// 2. Sleep for `collection_duration_seconds` (blocking the script).
    /// 3. Stop the listener and calculate the result.
    /// 4. Apply the result to the device if successful.
    #[pyo3(signature = (collection_duration_seconds, apply_timeout_seconds=5.0))]
    fn run_and_apply_magnetometer_calibration(
        &mut self,
        collection_duration_seconds: f64,
        apply_timeout_seconds: f64,
    ) -> PyResult<Option<bool>> {
        self.ensure_ok()?;

        let collection_duration = Duration::from_secs_f64(collection_duration_seconds);
        let apply_timeout = Duration::from_secs_f64(apply_timeout_seconds);

        // Call the inner Rust method we created earlier.
        let res = map_io(
            self.inner
                .run_and_apply_magnetometer_calibration(collection_duration, apply_timeout),
        )?;

        Ok(res)
    }

    #[pyo3(signature = (timeout_seconds=5.0))]
    fn cancel(&mut self, timeout_seconds: f64) -> PyResult<bool> {
        self.ensure_ok()?;

        let res = map_io(self.inner.cancel(Duration::from_secs_f64(timeout_seconds)))?;
        Ok(res.unwrap_or(false))
    }

    fn reboot(&mut self) -> PyResult<()> {
        self.ensure_ok()?;
        map_io(self.inner.reboot())?;
        Ok(())
    }

    fn is_running(&self) -> bool {
        self.inner.is_running()
    }

    fn __enter__(slf: Bound<'_, Self>) -> PyResult<Bound<'_, Self>> {
        slf.borrow_mut().start()?;
        Ok(slf)
    }

    fn __exit__(
        slf: Bound<'_, Self>,
        _exc_type: Option<Bound<'_, PyAny>>,
        _exc_value: Option<Bound<'_, PyAny>>,
        _traceback: Option<Bound<'_, PyAny>>,
    ) {
        let mut this = slf.borrow_mut();

        let _ = this.inner.stop_mock_log_stream(true, true);

        this.inner.stop();
    }
}

#[pymethods]
impl MockDeviceHandle {
    fn inject_response(&self, identifier: u16, payload: Vec<u8>) {
        let packet = FramedPacket::new(PacketHeader::Response, identifier, payload);
        self.inner.inject_framed_packet(packet);
    }

    #[pyo3(signature = (timeout_seconds))]
    fn wait_for_command_identifier(&self, timeout_seconds: f64) -> PyResult<Option<u16>> {
        map_io(
            self.inner
                .wait_for_command_identifier(Duration::from_secs_f64(timeout_seconds)),
        )
    }
}

#[pymodule(gil_used = false)]
fn firm_client(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<FIRMClient>()?;
    m.add_class::<MockDeviceHandle>()?;
    m.add_class::<ProcessedFIRMData>()?;
    m.add_class::<DeviceProtocol>()?;
    m.add_class::<DeviceInfo>()?;
    m.add_class::<DeviceConfig>()?;
    m.add_class::<CalibrationValues>()?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
