import struct
import pandas as pd
import numpy as np
import sys
import os



def decode(path):
    with open(path, 'rb') as f:

        characters = f.read()
        currentByte = 0

        bmp581_data = []
        icm45686_data = []
        mmc5983ma_data = []

        # identifier bytes for each packet type
        BMP581_ID = 0x42
        ICM45686_ID = 0x49 
        MMC5983MA_ID = 0x4D

        # struct sizes in bytes (not counting timestamp and id bytes)
        BMP581_SIZE = 2*4
        ICM45686_SIZE = 6*4
        MMC5983MA_SIZE = 3*4
        HEADER_SIZE = 15

        # number of times in a row whitespace has been repeated in the log
        num_repeat_whitespace = 0

        currentByte += (HEADER_SIZE + 1)

        # wrap in try/except, in case the end of file is reached
        try:
            while num_repeat_whitespace <= (max([BMP581_SIZE, ICM45686_SIZE, MMC5983MA_SIZE]) + 4):
                # whitespace, skip
                if characters[currentByte] == 0 :
                    currentByte += 1
                    num_repeat_whitespace += 1
                    continue
                
                # read timestamp
                num_repeat_whitespace = 0
                timestamp = struct.unpack('>I', b'\x00' + characters[currentByte + 1 : currentByte + 4])[0]
                currentByte += 4
                if characters[currentByte - 4] == BMP581_ID:
                    data = struct.unpack('<ff', characters[currentByte: currentByte + BMP581_SIZE])

                    bmp581_data.append([timestamp, data[0], data[1]])
                    currentByte += BMP581_SIZE

                elif characters[currentByte - 4] == ICM45686_ID:
                    data = struct.unpack('<ffffff', characters[currentByte : currentByte + ICM45686_SIZE])

                    icm45686_data.append([timestamp, data[0], data[1], data[2], data[3], data[4], data[5]])
                    currentByte += ICM45686_SIZE

                elif characters[currentByte - 4] == MMC5983MA_ID:
                    data = struct.unpack('<fff', characters[currentByte : currentByte + MMC5983MA_SIZE])

                    mmc5983ma_data.append([timestamp, data[0], data[1], data[2]])
                    currentByte += MMC5983MA_SIZE

                else:
                    # garbage data, end of file
                    print("hit garbage data, assuming end of file")
                    break
        except:
            # if end of file is reached, wont have enough bytes to decode the requested amount
            # so just skip that packet since it's incomplete, and begin converting to csv file
            pass

        # make each sensor data list as separate df
        bmp581_df = pd.DataFrame(bmp581_data, columns=['timestamp', 'temperature', 'pressure'])
        icm45686_df = pd.DataFrame(icm45686_data, columns=['timestamp', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z'])
        mmc5983ma_df = pd.DataFrame(mmc5983ma_data, columns=['timestamp', 'mag_x', 'mag_y', 'mag_z'])

        # write to csv
        bmp581_df.to_csv("BMP581_data.csv")
        icm45686_df.to_csv("ICM45686_data.csv")
        mmc5983ma_df.to_csv("MMC5983MA_data.csv")

if __name__ == "__main__":
    if len(sys.argv) == 2:
        path = sys.argv[1]
        if os.path.exists(path):
            decode(path)
        else:
            print("Invalid path to file")
    else:
        print("Specify path of file to decode")