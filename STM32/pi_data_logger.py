#!/usr/bin/env python3
"""
Data logger for STM32 sensor packets
Logs data to CSV file with timestamps
"""

import csv
import time
from datetime import datetime
from pathlib import Path
from pi_i2c_reader import STM32I2CReader, SensorPacket


class SensorDataLogger:
    """Logs sensor data to CSV file"""
    
    def __init__(self, output_dir: str = "sensor_logs"):
        """
        Initialize data logger
        
        Args:
            output_dir: Directory to store log files
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        self.current_file = None
        self.csv_writer = None
        self.file_handle = None
    
    def _create_log_file(self) -> Path:
        """Create a new log file with timestamp"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"sensor_data_{timestamp}.csv"
        filepath = self.output_dir / filename
        
        # Open file and create CSV writer
        self.file_handle = open(filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.file_handle)
        
        # Write header row
        self.csv_writer.writerow([
            'system_time',
            'timestamp_sec',
            'temperature_c',
            'pressure_pa',
            'accel_x_ms2',
            'accel_y_ms2',
            'accel_z_ms2',
            'gyro_x_rads',
            'gyro_y_rads',
            'gyro_z_rads',
            'mag_x_ut',
            'mag_y_ut',
            'mag_z_ut',
            'crc_valid'
        ])
        
        self.file_handle.flush()
        self.current_file = filepath
        
        return filepath
    
    def log_packet(self, packet: SensorPacket):
        """
        Log a sensor packet to CSV
        
        Args:
            packet: SensorPacket to log
        """
        if self.csv_writer is None:
            self._create_log_file()
        
        # Get system timestamp
        system_time = datetime.now().isoformat()
        
        # Write data row
        self.csv_writer.writerow([
            system_time,
            f"{packet.timestamp:.3f}",
            f"{packet.temperature:.2f}",
            f"{packet.pressure_raw:.2f}",
            f"{packet.accel_x:.6f}",
            f"{packet.accel_y:.6f}",
            f"{packet.accel_z:.6f}",
            f"{packet.gyro_x:.6f}",
            f"{packet.gyro_y:.6f}",
            f"{packet.gyro_z:.6f}",
            f"{packet.mag_x:.6f}",
            f"{packet.mag_y:.6f}",
            f"{packet.mag_z:.6f}",
            packet.crc_valid
        ])
        
        self.file_handle.flush()
    
    def close(self):
        """Close the current log file"""
        if self.file_handle:
            self.file_handle.close()
            self.file_handle = None
            self.csv_writer = None
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


def main():
    """Main logging loop"""
    print("="*60)
    print("STM32 Sensor Data Logger")
    print("="*60)
    
    reader = STM32I2CReader(bus_number=1)
    
    with SensorDataLogger(output_dir="sensor_logs") as logger:
        log_file = logger._create_log_file()
        print(f"\nLogging to: {log_file}")
        print(f"Press Ctrl+C to stop logging\n")
        
        packet_count = 0
        error_count = 0
        start_time = time.time()
        
        try:
            while True:
                packet = reader.read_packet()
                
                if packet:
                    logger.log_packet(packet)
                    packet_count += 1
                    
                    # Print status every 10 packets
                    if packet_count % 10 == 0:
                        elapsed = time.time() - start_time
                        rate = packet_count / elapsed if elapsed > 0 else 0
                        print(f"Logged {packet_count} packets "
                              f"({rate:.1f} Hz, {error_count} errors)")
                else:
                    error_count += 1
                
                # Read at ~10 Hz
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print(f"\n\nLogging stopped by user")
            elapsed = time.time() - start_time
            print(f"\nStatistics:")
            print(f"  Total packets: {packet_count}")
            print(f"  Errors: {error_count}")
            print(f"  Duration: {elapsed:.1f}s")
            print(f"  Average rate: {packet_count/elapsed:.1f} Hz")
            print(f"  Log file: {log_file}")


if __name__ == "__main__":
    main()
