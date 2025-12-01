import math
import os
import sys

import pandas as pd


def calibrate(path):
    # columns to use
    applicable_fields = ["accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"]

    read_csv = pd.read_csv(path, skipinitialspace=True, usecols=applicable_fields)

    field_element_dict = {}

    # extract data
    for key in read_csv:
        element_list = []
        for value in read_csv[key]:
            if math.isnan(value):
                continue
            if key == "accel_z":
                element_list.append(float(value) - 1.0)  # subtract gravity, expected 1gs
            else:
                element_list.append(float(value))
        field_element_dict[key] = element_list

    # calculate averages
    averages_dict = {key: sum(values) / len(values) for key, values in field_element_dict.items()}
    for key in read_csv:
        print(f"{key}: {averages_dict[key]}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Specify path of imu file to calibrate")
        sys.exit(1)
    path = sys.argv[1]
    if os.path.exists(path):
        calibrate(path)
    else:
        print("Invalid path to file")
