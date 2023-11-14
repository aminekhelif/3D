import re
from math import radians, sin, cos
import numpy as np

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

def quaternion_to_matrix(quaternion):
    """
    Convert a quaternion to a rotation matrix.
    """
    q0, q1, q2, q3 = quaternion
    print(q0, q1, q2, q3)
    return np.array([
        [1 - 2*q2*q2 - 2*q3*q3, 2*q1*q2 - 2*q3*q0, 2*q1*q3 + 2*q2*q0],
        [2*q1*q2 + 2*q3*q0, 1 - 2*q1*q1 - 2*q3*q3, 2*q2*q3 - 2*q1*q0],
        [2*q1*q3 - 2*q2*q0, 2*q2*q3 + 2*q1*q0, 1 - 2*q1*q1 - 2*q2*q2]
    ])

def check_internal_consistency(data):
    """
    Check if the data within the log is internally consistent.
    """
    inconsistencies = []
    print("before for loop")
    for i in range(1, len(data)):
        prev_entry = data[i - 1]
        current_entry = data[i]

        # Check position consistency
        expected_position = prev_entry["position"] + prev_entry["velocity"] * current_entry["delta_t"]
        if not np.allclose(expected_position, current_entry["new_position"], atol=1e-6):
            inconsistencies.append((current_entry["time"], "Position inconsistency"))

        # Check orientation consistency
        angular_velocity = current_entry["sensor_data"][1:4]  # wx, wy, wz
        angle = np.linalg.norm(angular_velocity) * current_entry["delta_t"]
        if angle != 0:
            axis = angular_velocity / np.linalg.norm(angular_velocity)
            expected_quaternion = np.array([
                cos(radians(angle / 2)),
                axis[0] * sin(radians(angle / 2)),
                axis[1] * sin(radians(angle / 2)),
                axis[2] * sin(radians(angle / 2))
            ])
            expected_matrix = quaternion_to_matrix(expected_quaternion)
            actual_matrix = quaternion_to_matrix(current_entry["new_current_orientation"])
            print(current_entry["new_current_orientation"])
            
            if not np.allclose(expected_matrix, actual_matrix, atol=1e-6):
                inconsistencies.append((current_entry["time"], "Orientation inconsistency"))

    return inconsistencies


# Sample code to read log content, parse it, and check for inconsistencies

# Replace this with the path to your actual log file
log_file_path = './Data.txt'

# Read the log file content
with open(log_file_path, 'r') as file:
    log_content = file.read()

# Parse the log data
parsed_data = parse_log_data(log_content)

# Check for internal consistency
inconsistencies = check_internal_consistency(parsed_data)

# Print the inconsistencies
for inconsistency in inconsistencies:
    print(f"Time: {inconsistency[0]}, Issue: {inconsistency[1]}")
    
else:
    print("Test Passed: No inconsistencies found.")
