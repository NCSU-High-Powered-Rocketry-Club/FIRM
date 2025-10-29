import struct
import pandas as pd
import sys
import os
import struct
import sys

import pandas as pd


# identifier for each packet type
BMP581_ID = 'B'
ICM45686_ID = 'I'
MMC5983MA_ID = 'M'

# struct sizes in bytes (not counting timestamp and id bytes)
BMP581_SIZE = 6
ICM45686_SIZE = 15
MMC5983MA_SIZE = 7
HEADER_SIZE_TEXT = 14 # size of the "FIRM LOG vx.x" text
HEADER_NUM_FLOATS = 5 # number of scale factor floats in the header

def twos_complement(val, bits):
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val

class Decoder:
    bmp581_scale_factors = [0, 0] # temp, pressure
    icm45686_scale_factors = [0, 0] # acc, gyro
    mmc5983ma_scale_factor = 0 # magnetic field

    # the data for each sensor
    bmp581_data = []
    icm45686_data = []
    mmc5983ma_data = []

    # because we use clock cycle count for timestamp, and we expect the cycle count to
    # overflow every ~0.1 seconds, we handle the overflow in this file to make the timestamp
    # column continuous.
    timestamp_seconds = 0
    last_clock_count = 0

    # number of times in a row whitespace has been repeated in the log
    num_repeat_whitespace = 0

    def __init__(self, file):
        self.f = file
        self.read_header(self.f)

    def get_delta_timestamp(self, next_clock_count):
        # if the clock cycle count loops around due to unsigned int overflow, the difference
        # is found by adding the size of the integer used to store the count
        if (next_clock_count < self.last_clock_count):
            next_clock_count += (2**24)
        return next_clock_count - self.last_clock_count

    def read_packet(self):
        try:
            # read packet ID
            id_byte = self.f.read(1)[0]
            if id_byte == 0:
                # whitespace, skip
                self.num_repeat_whitespace += 1
                # end of data if whitespace repeats enough times
                if self.num_repeat_whitespace > max([BMP581_SIZE, ICM45686_SIZE, MMC5983MA_SIZE]) + 4:
                    return False
                return True
            self.num_repeat_whitespace = 0

            # read timestamp
            clock_count_bytes = self.f.read(3)
            clock_count = struct.unpack('>I', b'\x00' + clock_count_bytes)[0]
            self.timestamp_seconds += (self.get_delta_timestamp(clock_count)) / 168e6
            self.last_clock_count = clock_count

            if id_byte == ord(BMP581_ID):
                bytes = self.f.read(BMP581_SIZE)
                data = self.convert_bmp581(bytes)
                self.bmp581_data.append(data)
                return True
            if id_byte == ord(ICM45686_ID):
                bytes = self.f.read(ICM45686_SIZE)
                data = self.convert_icm45686(bytes)
                self.icm45686_data.append(data)
                return True
            if id_byte == ord(MMC5983MA_ID):
                bytes = self.f.read(MMC5983MA_SIZE)
                data = self.convert_mmc5983ma(bytes)
                self.mmc5983ma_data.append(data)
                return True

            # if not an ID byte, most likely garbage data at end of file
            return False
        except:
            print("error")
            # hit end of file
            return False

    def read_header(self, file):
        file.read(HEADER_SIZE_TEXT)
        scale_factor_bytes = file.read(HEADER_NUM_FLOATS * 4)
        scale_factors = struct.unpack('<fffff', scale_factor_bytes)
        self.bmp581_scale_factors = scale_factors[0 : 2]
        self.icm45686_scale_factors = scale_factors[2 : 4]
        self.mmc5983ma_scale_factor = scale_factors[4]

    def convert_bmp581(self, binary_packet):
        temp_pressure = struct.unpack('<II', binary_packet[0 : 3] + b'\00' + binary_packet[3 : 6] + b'\00')
        data = [
            self.timestamp_seconds,
            temp_pressure[0] / self.bmp581_scale_factors[0],
            temp_pressure[1] / self.bmp581_scale_factors[1],
        ]
        return data

    def convert_icm45686(self, binary_packet):
        accel_x_bin = (binary_packet[0] << 12) | (binary_packet[1] << 4) | (binary_packet[12] >> 4)
        accel_y_bin = (binary_packet[2] << 12) | (binary_packet[3] << 4) | (binary_packet[13] >> 4)
        accel_z_bin = (binary_packet[4] << 12) | (binary_packet[5] << 4) | (binary_packet[14] >> 4)

        gyro_x_bin = (binary_packet[6] << 12) | (binary_packet[7] << 4) | (binary_packet[12] & 0x0F)
        gyro_y_bin = (binary_packet[8] << 12) | (binary_packet[9] << 4) | (binary_packet[13] & 0x0F)
        gyro_z_bin = (binary_packet[10] << 12) | (binary_packet[11] << 4) | (binary_packet[14] & 0x0F)

        accel_x_bin = twos_complement(accel_x_bin, 20)
        accel_y_bin = twos_complement(accel_y_bin, 20)
        accel_z_bin = twos_complement(accel_z_bin, 20)
        gyro_x_bin = twos_complement(gyro_x_bin, 20)
        gyro_y_bin = twos_complement(gyro_y_bin, 20)
        gyro_z_bin = twos_complement(gyro_z_bin, 20)

        data = [
            self.timestamp_seconds,
            accel_x_bin / self.icm45686_scale_factors[0],
            accel_y_bin / self.icm45686_scale_factors[0],
            accel_z_bin / self.icm45686_scale_factors[0],
            gyro_x_bin / self.icm45686_scale_factors[1],
            gyro_y_bin / self.icm45686_scale_factors[1],
            gyro_z_bin / self.icm45686_scale_factors[1],
        ]
        return data

    def convert_mmc5983ma(self, binary_packet):
        mag_x_bin = (binary_packet[0] << 10) | (binary_packet[1] << 2) | (binary_packet[6] >> 6)
        mag_y_bin = (binary_packet[2] << 10) | (binary_packet[3] << 2) | ((binary_packet[6] & 0x30) >> 4)
        mag_z_bin = (binary_packet[4] << 10) | (binary_packet[5] << 2) | ((binary_packet[6] & 0x0C))

        data = [
            self.timestamp_seconds,
            mag_x_bin / self.mmc5983ma_scale_factor,
            mag_y_bin / self.mmc5983ma_scale_factor,
            mag_z_bin / self.mmc5983ma_scale_factor,
        ]
        return data

def decode(path):
    with open(path, 'rb') as f:
        decoder = Decoder(f)
        while (decoder.read_packet()):
            continue

        bmp581_df = pd.DataFrame(decoder.bmp581_data, columns=['timestamp', 'temperature', 'pressure'])
        icm45686_df = pd.DataFrame(decoder.icm45686_data, columns=['timestamp', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z'])
        mmc5983ma_df = pd.DataFrame(decoder.mmc5983ma_data, columns=['timestamp', 'mag_x', 'mag_y', 'mag_z'])

        # write to csv
        bmp581_df.to_csv("BMP581_data.csv", index=False)
        icm45686_df.to_csv("ICM45686_data.csv", index=False)
        mmc5983ma_df.to_csv("MMC5983MA_data.csv", index=False)

if __name__ == "__main__":
    if len(sys.argv) == 2:
        path = sys.argv[1]
        if os.path.exists(path):
            decode(path)
        else:
            print("Invalid path to file")
    else:
        print("Specify path of file to decode")
        sys.exit(1)
