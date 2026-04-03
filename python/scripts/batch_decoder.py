import os
import sys
from data_analyzer import is_launch_log
import decoder

# iterates through all .FRM files in the folder, uses data_analyzer to make list of launcgh logs
def find_launch_logs(folder_path, min_pressure_threshold = 1000):
    print(f"Finding launch logs in: {folder_path}")
    launch_logs = []

    for filename in os.listdir(folder_path): #getting only .frm files in the folder.
        if filename.endswith(".FRM"):
            file_path = os.path.join(folder_path, filename)
            if is_launch_log(file_path, min_pressure_threshold):
                launch_logs.append(filename)
    if len(launch_logs) == 0:
        print("No launch logs found.\nTry adjusting the minimum pressure threshold (default 1000) for determining launch logs with the optional second argument:\n python batch_decoder.py <folder_path> <min_pressure_threshold>")
    else:
        print(f"Launch logs found: {launch_logs}")
    return launch_logs

# iterates through all .FRM files in the folder, uses data_analyzer to determine if they are launch logs, and decodes them if they are.
def decode_launch_logs(folder_path, min_pressure_threshold = 1000):
    launch_logs = find_launch_logs(folder_path, min_pressure_threshold)
    if len(launch_logs) == 0:
        return
    for log in launch_logs:
        file_path = os.path.join(folder_path, log)
        decoder.decode(file_path)

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        folder_path = sys.argv[1]
        if len(sys.argv) == 3: # optional threshhold argument for determining launch logs.
            min_pressure_threshold = int(sys.argv[2])
        else:            
            min_pressure_threshold = 1000
        if os.path.exists(folder_path):
            decode_launch_logs(folder_path, min_pressure_threshold)
        else:
            print("Invalid path to folder")

    else:
        print("Specify path of folder containing log files")
        sys.exit(1)