import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import time


# TODO - Add comments to code
# TODO - Add docstrings to functions
# TODO - Add unit tests
# TODO - Add logging
# TODO - Add error handling
# TODO - Add command line arguments
# TODO - Add requirements.txt
# TODO - Add setup.py for initial setup
# TODO - Handle the case when the sensor data is not available
# TODO - Add a function to read sensor data from a file
# TODO - Add a function to read sensor data from a serial port
# TODO - Add a ²function to read sensor data from a socket
# TODO - Add a function to read sensor data from a ROS topic
# TODO - Add a function to read sensor data from a ROS bag
# TODO - Handle the case where initial gyro angles are not zero
# TODO - Handle the case where initial velocity is not zero

class Quaternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __mul__(self, other):
        w = self.w*other.w - self.x*other.x - self.y*other.y - self.z*other.z
        x = self.w*other.x + self.x*other.w + self.y*other.z - self.z*other.y
        y = self.w*other.y - self.x*other.z + self.y*other.w + self.z*other.x
        z = self.w*other.z + self.x*other.y - self.y*other.x + self.z*other.w
        return Quaternion(w, x, y, z)

    def normalize(self):
        norm = np.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        self.w /= norm
        self.x /= norm
        self.y /= norm
        self.z /= norm
        return self

    def conjugate(self):
        return Quaternion(self.w, -self.x, -self.y, -self.z)

def gyro_to_quaternion(gyro_data, delta_t):
    norm_gyro = np.linalg.norm(gyro_data)
    if norm_gyro > 0:
        axis = gyro_data / norm_gyro
    else:
        axis = np.array([0, 0, 0])
    theta_over_two = norm_gyro * delta_t / 2.0
    w = np.cos(theta_over_two)
    x, y, z = axis * np.sin(theta_over_two)
    return Quaternion(w, x, y, z)

def rotate_vector_by_quaternion(vector, quat):
    vector_quat = Quaternion(0, *vector)
    rotated_vector_quat = quat * vector_quat * quat.conjugate()
    return np.array([rotated_vector_quat.x, rotated_vector_quat.y, rotated_vector_quat.z])

# Placeholder for sensor data reading, replace with actual reading logic
def read_sensor_data():
    time = np.arange(0, 100, 0.1)
    # gyro_x = np.sin(time)
    # gyro_y = np.cos(time)
    # gyro_z = np.arange(0, 10, 0.01)
    # acc_x = np.arange(0, 10, 0.01)
    # acc_y = np.sin(time)
    # acc_z = np.array([9.81]*len(time))
    gyro_x = np.zeros(len(time))
    gyro_y = np.zeros(len(time))
    gyro_z = np.zeros(len(time))
    acc_x = np.ones(len(time))
    acc_y = np.array([2]*len(time))
    acc_z = np.array([9.81]*len(time))
    return np.vstack((time, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z)).T

def process_sensor_data(sensor_data, last_time, current_orientation):
    current_time = sensor_data[0]
    delta_t = current_time - last_time
    
    gyro_data = sensor_data[1:4]
    accel_data = sensor_data[4:7]

    gyro_quat = gyro_to_quaternion(gyro_data, delta_t)
    current_orientation *= gyro_quat
    current_orientation.normalize()

    gravity_vector = [0, 0, 9.81]
    gravity_in_sensor_frame = rotate_vector_by_quaternion(gravity_vector, current_orientation.conjugate())
    linear_accel = accel_data - gravity_in_sensor_frame

    return current_orientation, linear_accel, delta_t

def plot_trajectories(*all_positions, labels=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Define different markers for each trajectory
    markers = ['o', '^', 's', 'p', '*', '+', 'x', 'd']
    
    for idx, positions in enumerate(all_positions):
        color = f"C{idx}"  # This picks a unique color from matplotlib's color cycle
        marker = markers[idx % len(markers)]  # Cycle through different markers
        label = f"Trajectory {idx + 1}" if labels is None or idx >= len(labels) else labels[idx]
        ax.plot(positions[:,0], positions[:,1], positions[:,2], color=color, marker=marker, label=label)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()  # Always display the legend
    
    plt.show()
    
def integrate_acceleration(acceleration, velocity, position, delta_t,new):
    """
    Integrates acceleration to update velocity and position.
    Assumes constant acceleration over the small interval delta_t.
    """
    new_velocity = velocity + acceleration * delta_t
    if new:
        new_position = position + new_velocity * delta_t + 0.5 * acceleration * delta_t**2
    else:
        new_position = position + velocity * delta_t + 0.5 * acceleration * delta_t**2
    
    
    return new_velocity, new_position

    
def integrate_velocity(acceleration, velocity, delta_t):
    """
    Update velocity by integrating acceleration over time.
    Using the trapezoidal rule for numerical integration.
    """
    # Assuming acceleration is constant over the small interval delta_t
    return velocity + acceleration * delta_t

def integrate_position(velocity, position, delta_t):
    """
    Update position by integrating velocity over time.
    Using the trapezoidal rule for numerical integration.
    """
    # Assuming velocity is constant over the small interval delta_t
    return position + velocity * delta_t

def init_plot():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    line, = ax.plot([], [], [], 'r-')  # 'r-' is the color and line style
    ax.set_xlim(-10, 10)  # Set the limits of the plot here
    ax.set_ylim(-10, 10)
    ax.set_zlim(-10, 10)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    return fig, ax, line

# This function will be called for each frame to update the plot
def update_plot(frame, positions_list, line):
    # Simulate data acquisition
    sensor_data = read_sensor_data()  # This should return the latest frame of sensor data
    current_time = sensor_data[0]
    
    # Initial conditions if this is the first frame
    if not positions_list:
        last_time = current_time
        velocity = np.array([0.0, 0.0, 0.0])
        position = np.array([0.0, 0.0, 0.0])
        current_orientation = Quaternion(1, 0, 0, 0)
    else:
        last_time = positions_list[-1][0]
        velocity = positions_list[-1][1]
        position = positions_list[-1][2]
        current_orientation = positions_list[-1][3]

    delta_t = current_time - last_time
    
    # Process sensor data to get current orientation and linear acceleration
    current_orientation, linear_accel, _ = process_sensor_data(sensor_data, last_time, current_orientation)
    
    # Integrate acceleration to get new velocity and position
    velocity, new_position = integrate_acceleration(linear_accel, velocity, position, delta_t, new=False)
    
    # Append the new position and other data to the list
    positions_list.append((current_time, velocity, new_position, current_orientation))
    
    # Update the line data for the plot
    xs, ys, zs = zip(*[pos[2] for pos in positions_list])
    line.set_data(np.array([xs, ys]))
    line.set_3d_properties(np.array(zs))
    
    return line,


def main():
    fig, ax, line = init_plot()
    positions_list = []  # This will store all the data for plotting

    # Create the animation object
    ani = FuncAnimation(fig, update_plot, fargs=(positions_list, line), init_func=lambda: line, blit=False, interval=100)

    # Show the plot
    plt.show()
    
if __name__ == "__main__":
    main()
