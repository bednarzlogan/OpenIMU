import os
from datetime import datetime
import numpy as np

"""
In this script, we'll match GNSS PV data with IMU data by checking
the IMU data timestamps which is closest to the GNSS position information
"""

PATH_TO_GNSS = os.path.join(".","waypoints_20250620_221313.csv")
PATH_TO_IMU = os.path.join(".","simulated_logs","ground_truth_2025_06_18_21_54_51.csv")
processing_start_time: str = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
MERGED_FILE_NAME = f"matched_gnss_imu_{processing_start_time}.csv"

# max allowed position delta in meters before considering a new GNSS point
MAX_POSITION_JUMP = 0.5  # meters
# max allowed time delta in seconds between GNSS and IMU
MAX_TIME_DIFF = 0.5  # seconds

def main() -> None:
    # open up the relevant files
    if not os.path.exists(PATH_TO_GNSS) or not os.path.exists(PATH_TO_IMU):
        print(f"Missing files: {PATH_TO_GNSS} or {PATH_TO_IMU}")
        return

    # match the positions and timestamps in ground truth
    with open(PATH_TO_GNSS, 'r') as gnss_file, open(PATH_TO_IMU, 'r') as imu_file:
        def mag_dist(data1: tuple[float, float], 
                     data2: tuple[float, float]) -> float:
            """
            Calculate the Euclidean distance between two points
            """
            return ((data1[0] - data2[0]) ** 2 + (data1[1] - data2[1]) ** 2) ** 0.5

        # form merged data header from gnss data header
        gnss_header: str = gnss_file.readline()
        if not gnss_header:
            print("GNSS file is empty or missing header.")
            return

        # write the header to the merged file
        with open(MERGED_FILE_NAME, 'w') as output_file:
            output_file.write(f"timestamp,{gnss_header.strip()}\n")

        # we'll hold all the IMU data in memory, since we'll need to hop around in it
        imu_data = imu_file.readlines()[1:]  # skip the header line
        cur_line_imu: int = 0

        # now, we can start our processing
        print("Starting GNSS-IMU matching...")
        offset_tolerance = 0.5  # meters offset tolerance for matching
        last_timestamp = -1.0
        last_gnss_position: tuple[float, float] = (-1e32, -1e32)
        while True:
            # read the next line from the GNSS file
            gnss_line = gnss_file.readline()
            if not gnss_line:
                break

            gnss_tokens = gnss_line.strip().split(',')
            try:
                en_position_target = (float(gnss_tokens[0]), float(gnss_tokens[1]))
                #  gnss_time = float(gnss_tokens[-1])  # not in the logs yet
            except (ValueError, IndexError):
                print(f"Skipping malformed GNSS line: {gnss_tokens}")
                continue

            # compute distance between last accepted GNSS point and current candidate
            if last_gnss_position != (-1e32, -1e32):
                dist = np.linalg.norm(np.array(en_position_target) - np.array(last_gnss_position))
                if dist < MAX_POSITION_JUMP:
                    # too similar, skip GNSS update — IMU hasn't moved enough
                    continue

            last_gnss_position = en_position_target

            # find the closest point in the IMU data
            found_match: bool = False
            while True:
                # move to the next IMU line
                cur_line_imu += 1
                if cur_line_imu >= len(imu_data):
                    print("Reached end of IMU data without finding a matching GNSS position.")
                    raise StopIteration("No more IMU data to process.")
                imu_line = imu_data[cur_line_imu]
                imu_tokens = imu_line.strip().split(',')
                try:
                    imu_timestamp = float(imu_tokens[0])
                    imu_en_position = (float(imu_tokens[1]), float(imu_tokens[2]))
                except (ValueError, IndexError):
                    continue

                # don't have timestamps in the logs yet, so we can't check time difference
                # if abs(gnss_time - imu_timestamp) > MAX_TIME_DIFF:
                #     continue

                if mag_dist(en_position_target, imu_en_position) <= offset_tolerance:
                    found_match = True
                    break

            if found_match:
                # print the matched data
                print(f"Matched GNSS: {en_position_target} with IMU: {imu_en_position} at time {imu_timestamp}")
                last_timestamp = imu_timestamp

                with open(MERGED_FILE_NAME, 'a') as output_file:
                    # append found timestamp to original GNSS line
                    output_file.write(f"{last_timestamp},{gnss_line}")  # NOTE: there's already '\n' at the end of gnss_line


if __name__ == "__main__":
    main()
    print("Program completed successfully.")