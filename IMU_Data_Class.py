# Restructuring the provided code into a more organized, class-based format
from typing import Any
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import time
import logging
import os
from Data_Reader import read_sensor_data

# Todo handle magnetometre later

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


# TODO MAKE PROCESS DATA OUTPUTS ALL NECESSARY DATA / POSIITION QUATRINION AXIS VELOCITY
# TODO MAKE VISUALIZER UPDATE independently of processor
# TODO add reference quiver to visualizer
#  TODO fix gravity vector
#  TODO fix acceleration integration and in wich axis it is integrated - take note of current quatrenion
#  TODO fix latency bitween sensor data and visualizer
#  TODO fix latency bitween sensor data from socket into processor and visualizer
#  TODO optimize code
#  TODO opptimize calculations
#  TODO ADD integrate_acceleration to processor
#  TODO manage read sensor data function when takes frame argument and when not
#  TODO find a better plotter smoother and faster
#  TODO integrate EKF kalman filter

# 

# Set up basic logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class Quaternion:
    """
    A class representing a quaternion for 3D rotation.
    """
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __mul__(self, other):
        # Quaternion multiplication
        w = self.w*other.w - self.x*other.x - self.y*other.y - self.z*other.z
        x = self.w*other.x + self.x*other.w + self.y*other.z - self.z*other.y
        y = self.w*other.y - self.x*other.z + self.y*other.w + self.z*other.x
        z = self.w*other.z + self.x*other.y - self.y*other.x + self.z*other.w
        return Quaternion(w, x, y, z)

    def normalize(self):
        # Normalize the quaternion
        norm = np.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        self.w /= norm
        self.x /= norm
        self.y /= norm
        self.z /= norm
        return self

    def conjugate(self):
        # Conjugate of the quaternion
        return Quaternion(self.w, -self.x, -self.y, -self.z)
    
    def __str__(self) -> str:
        return f"Quaternion({self.w}, {self.x}, {self.y}, {self.z})"
    
    def __getattribute__(self, __name: str) -> Any:
        return super().__getattribute__(__name)
    def __format__(self, __format_spec: str) -> str:
        return super().__format__(__format_spec)
    
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
    quat.normalize()
    vector_quat = Quaternion(0, *vector)
    rotated_vector_quat = quat * vector_quat * quat.conjugate()
    return np.array([rotated_vector_quat.x, rotated_vector_quat.y, rotated_vector_quat.z])

def quatrenion_to_vector(quat):
    quat.normalize()
    return np.array([quat.x, quat.y, quat.z])


class SensorDataProcessor:
    """
    A class to process sensor data using IMU and quaternion calculations.
    """
    def __init__(self):
        # self.current_orientation = Quaternion(1, 0, 0, 0)
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, 0.0])
        self.accelerations_history = []

    def process_sensor_data(self,current_orientation,current_axis, sensor_data, last_time):
        # logging.info("Processing sensor data...")
        current_time = sensor_data[0]
        delta_t = current_time - last_time
        
        gyro_data = sensor_data[1:4]
        accel_data = sensor_data[4:7]
        x_axis , y_axis , z_axis = current_axis

        gyro_quat = gyro_to_quaternion(gyro_data, delta_t)
        # print("gyro_quat : ",gyro_quat)
        # print("gyro_data : ",gyro_data)
        # print("current_orientation : ",current_orientation)
              
        current_orientation = current_orientation * gyro_quat
        # print("current_orientation : ",current_orientation)
        x_axis = rotate_vector_by_quaternion(x_axis, gyro_quat)
        y_axis = rotate_vector_by_quaternion(y_axis, gyro_quat)
        z_axis = rotate_vector_by_quaternion(z_axis, gyro_quat)
        current_axis = [x_axis,y_axis,z_axis]
        # print("current_axis : ",current_axis)
        # print("----------------------------------------\n")
        
        

  
        gravity_vector = [0, 0, -9.88]
        gravity_in_sensor_frame = rotate_vector_by_quaternion(gravity_vector, current_orientation.conjugate())
        linear_accel = accel_data #- gravity_in_sensor_frame

        return current_axis,current_orientation,linear_accel, delta_t

    def integrate_acceleration(self, linear_accel, delta_t):
        self.accelerations_history.append(linear_accel)
        if len(self.accelerations_history) > 3:
            self.accelerations_history.pop(0)  # Keep only the latest 3 accelerations

        velocity, new_position = integrate_acceleration(self.accelerations_history, self.velocity, self.position, delta_t)
        self.velocity = velocity
        self.position = new_position

        return velocity, new_position



class IMUVisualizer:
    def __init__(self, processor=SensorDataProcessor()):
        self.processor = processor
        self.fig, self.ax, self.line = self.init_plot()
        self.positions_list = []
        self.quiver_objects = []
        self.accelerations_history = []
        self.xyzaxis = [np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])]
        self.frame_counter = {'count': 0}
        self.current_orientation = Quaternion(1, 0, 0, 0)

    def init_plot(self):
        logging.info("Initializing the plot...")
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        line, = ax.plot([], [], [], 'r-')  # 'r-' is the color and line style
        line.set_data([], [])   
        line.set_3d_properties([]) 
        ax.set_xlim(-10, 10)  # Set the limits of the plot here
        ax.set_ylim(-10, 10)
        ax.set_zlim(-10, 10)
        ax.set_aspect('equal')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        return fig, ax, line

    def update_plot(self, frame):
        sensor_data = read_sensor_data(frame)  # This should return the latest frame of sensor data
        print("sensor_data = ",sensor_data)
        current_time = sensor_data[0]
        with open("./Data.txt", "a") as f:  
            f.write('Time : '+ str(current_time) +' Sensor Data : ' + str(sensor_data)+"\n")

        # Initial conditions if this is the first frame
        if not self.positions_list:
            last_time = current_time
            velocity = np.array([0.0, 0.0, 0.0])
            position = np.array([0.0, 0.0, 0.0])
            

        else:
            last_time = self.positions_list[-1][0]
            velocity = self.positions_list[-1][1]
            position = self.positions_list[-1][2]
            self.current_orientation = self.positions_list[-1][3] #current quaternion
            
            
        delta_t = current_time - last_time
        try:
            x_axis , y_axis , z_axis = self.xyzaxis
             
        except:
            x_axis = np.array([1, 0, 0])
            y_axis = np.array([0, 1, 0])
            z_axis = np.array([0, 0, 1])
            self.xyzaxis= [x_axis,y_axis,z_axis]
        
        with open("./Data.txt", "a") as f:
            f.write('delta_t : '+ str(delta_t) +"\n")
            f.write('velocity : '+ str(velocity) +"\n")
            f.write('position : '+ str(position) +"\n")
            # f.write('current_orientation : '+ str(current_orientation) +"\n")
        
        # Process sensor data to get current orientation and linear acceleration
        self.xyzaxis,self.current_orientation, linear_accel, _ = self.processor.process_sensor_data(self.current_orientation,self.xyzaxis,sensor_data, last_time)
        
        with open("./Data.txt", "a") as f:
            f.write('new_current_orientation : '+ str(self.current_orientation) +"\n")
            f.write('linear_accel : '+ str(linear_accel) +"\n")
        
        # Store the acceleration data for Simpson's method
        self.accelerations_history.append(linear_accel)
        if len(self.accelerations_history) > 3:
            self.accelerations_history.pop(0)  # Keep only the latest 3 accelerations

        # Integrate acceleration to get new velocity and position
        velocity, new_position = integrate_acceleration(self.accelerations_history, velocity, position, delta_t)
        
        with open("./Data.txt", "a") as f:
            f.write('new_velocity : '+ str(velocity) +"\n")
            f.write('new_position : '+ str(new_position) +"\n")
        # Append the new position and other data to the list
        self.positions_list.append((current_time, velocity, new_position, self.current_orientation))
        
        # Update the line data for the plot
        xs, ys, zs = zip(*[pos[2] for pos in self.positions_list])
        self.line.set_data(np.array([xs, ys]))
        self.line.set_3d_properties(np.array(zs))
        
        padding = 1.0  # You can adjust the padding as needed
        self.ax.set_xlim([min(xs) - padding, max(xs) + padding])
        self.ax.set_ylim([min(ys) - padding, max(ys) + padding])
        self.ax.set_zlim([min(zs) - padding, max(zs) + padding])
        self.ax.set_aspect('equal')
        
        try:
            x_axis , y_axis , z_axis = self.xyzaxis
             
        except:
            x_axis = np.array([1, 0, 0])
            y_axis = np.array([0, 1, 0])
            z_axis = np.array([0, 0, 1])
            
        with open("./Data.txt", "a") as f:
            f.write('x_axis : '+ str(x_axis) +"\n")
            f.write('y_axis : '+ str(y_axis) +"\n")
            f.write('z_axis : '+ str(z_axis) +"\n")
            
        # x_axis = rotate_vector_by_quaternion(x_axis, current_orientation)
        # y_axis = rotate_vector_by_quaternion(y_axis, current_orientation)
        # z_axis = rotate_vector_by_quaternion(z_axis, current_orientation)
        
        with open("./Data.txt", "a") as f:
            f.write('new_x_axis : '+ str(x_axis) +"\n")
            f.write('new_y_axis : '+ str(y_axis) +"\n")
            f.write('new_z_axis : '+ str(z_axis) +"\n")
            f.write("----------------------------------------\n")
        # Get the current limits of the plot
        x_limits = self.ax.get_xlim3d()
        range = abs(x_limits[1] - x_limits[0])

        
        # Update axes for the latest position
        if self.quiver_objects:
            for quiv in self.quiver_objects:
                quiv.remove()
        
        axis_length = range/5       
        quiver_x = self.ax.quiver(*new_position, *x_axis, color='r', length=axis_length, normalize=True)
        quiver_y = self.ax.quiver(*new_position, *y_axis, color='g', length=axis_length, normalize=True)
        quiver_z = self.ax.quiver(*new_position, *z_axis, color='b', length=axis_length, normalize=True)

        self.quiver_objects[:] = [quiver_x, quiver_y, quiver_z]
        self.xyzaxis[:] =[x_axis,y_axis,z_axis]
        
        return self.line, self.quiver_objects, self.xyzaxis



    def animate(self, i):
        if self.frame_counter['count'] >= 100:
            self.ani.event_source.stop()
            logging.info("Stopping the animation...")
            return self.line,

        updated_line, self.quiver_objects , self.xyzaxis = self.update_plot(i)
        
        self.frame_counter['count'] += 1
        return updated_line

    def start_animation(self):
        self.ani = FuncAnimation(self.fig, self.animate, frames=np.arange(1000), interval=100, blit=False)
        plt.show()
        



def integrate_acceleration(accelerations, velocity, position, delta_t):
    """
    Intégration de l'accélération pour mettre à jour la vitesse et la position.
    Utilise une méthode plus simple au début et passe à la méthode de Simpson lorsque suffisamment de données sont disponibles.
    """
    # logging.info("Integrating acceleration...")
    if len(accelerations) < 3:
        # Si moins de 3 points sont disponibles, utilisez une méthode plus simple
        # Par exemple, méthode du point milieu ou du trapèze
        new_velocity = velocity + accelerations[-1] * delta_t
        new_position = position + new_velocity * delta_t
    else:
        # Utilisation de la méthode de Simpson
        a_n_minus_1, a_n, a_n_plus_1 = accelerations[-3:]
        new_velocity = velocity + (delta_t / 3) * (a_n_minus_1 + 4 * a_n + a_n_plus_1)
        new_position = position + new_velocity * delta_t  # Ici, une méthode simple peut toujours être utilisée

    return new_velocity, new_position

       

# Main function
def main():
    if os.path.exists("./Data.txt"):
        os.remove("./Data.txt")
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    logging.info("Starting the animation...")
    processor = SensorDataProcessor()
    visualizer = IMUVisualizer(processor)
    visualizer.start_animation()

if __name__ == "__main__":
   main()