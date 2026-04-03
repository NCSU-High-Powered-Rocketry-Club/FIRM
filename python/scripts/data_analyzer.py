import pandas as pd
import sys
from decoder import Decoder

def is_launch_log(path, minimum_pressure_threshold = 1000):
    with open(path, 'rb') as f:
        decoder = Decoder(f)
        print(f"Reading file: {path}")

        # use decoder to read through the file. 
        while (decoder.read_packet()):
            continue

    # use barometer data to get pressure readings. 
    bmp581_df = pd.DataFrame(decoder.bmp581_data, columns=['timestamp', 'temperature', 'pressure'])
    pressureData = bmp581_df['pressure']
    min_pressure = pressureData.min()
    initial_pressure = pressureData.iloc[0]
    print(min_pressure, initial_pressure)


    if (initial_pressure - min_pressure) > minimum_pressure_threshold: # if pressure drops by more than 1000 Pa, it's likely a launch log. 
        return True
    else:
        return False

if __name__ == "__main__":
    path = sys.argv[1]
    is_launch_log(path)

