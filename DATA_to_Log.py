import re
from math import radians, sin, cos
import numpy as np
import time
import os
def parse_log_data(log_content):
    """
    Parse the log content to extract relevant data.
    """
    entries = re.findall(r"Time : (\d+\.\d+).*?Sensor Data : \[(.*?)\].*?delta_t : (\d+\.\d+).*?velocity : \[(.*?)\].*?position : \[(.*?)\].*?current_orientation : Quaternion\((.*?)\).*?new_current_orientation : Quaternion\((.*?)\).*?linear_accel : \[(.*?)\].*?new_velocity : \[(.*?)\].*?new_position : \[(.*?)\].*?x_axis : \[(.*?)\].*?y_axis : \[(.*?)\].*?z_axis : \[(.*?)\].*?new_x_axis : \[(.*?)\].*?new_y_axis : \[(.*?)\].*?new_z_axis : \[(.*?)\]", log_content, re.DOTALL)
    
    parsed_data = []
    for entry in entries:
        time, sensor_data, delta_t, velocity, position, current_orientation, new_current_orientation, linear_accel, new_velocity, new_position, x_axis, y_axis, z_axis, new_x_axis, new_y_axis, new_z_axis = entry

        parsed_data.append({
            "time": float(time),
            "sensor_data": np.fromstring(sensor_data, sep=' '),
            "delta_t": float(delta_t),
            "velocity": np.fromstring(velocity, sep=' '),
            "position": np.fromstring(position, sep=' '),
            "current_orientation": np.fromstring(current_orientation, sep=', '),
            "new_current_orientation": np.fromstring(new_current_orientation, sep=', '),
            "linear_accel": np.fromstring(linear_accel, sep=' '),
            "new_velocity": np.fromstring(new_velocity, sep=' '),
            "new_position": np.fromstring(new_position, sep=' '),
            "x_axis": np.fromstring(x_axis, sep=' '),
            "y_axis": np.fromstring(y_axis, sep=' '),
            "z_axis": np.fromstring(z_axis, sep=' '),
            "new_x_axis": np.fromstring(new_x_axis, sep=' '),
            "new_y_axis": np.fromstring(new_y_axis, sep=' '),
            "new_z_axis": np.fromstring(new_z_axis, sep=' ')
        })

    return parsed_data

if __name__ == "__main__":
    # Ensure the log directory exists
    log_directory = "./log"
    if not os.path.exists(log_directory):
        os.makedirs(log_directory)

    # Read data from the log file
    with open("./IMU.log", "r") as log_file:
        log_content = log_file.read()
        parsed_data = parse_log_data(log_content)

        # Write the parsed data to a new file
        timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        log_filename = os.path.join(log_directory, "log_" + timestamp + ".CSV")
        with open(log_filename, "w") as parsed_data_file:
            for entry in parsed_data:
                parsed_data_file.write(str(entry))
                parsed_data_file.write("\n")
                print("Wrote entry to file.")