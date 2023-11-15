# Restructuring the provided code into a more organized, class-based format
from typing import Any
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import time
import logging
import os
import json
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
    
    def to_dict(self):
        return {"w": self.w, "x": self.x, "y": self.y, "z": self.z}
    
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
        self.current_orientation = Quaternion(1, 0, 0, 0)
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, 0.0])
        self.xyzaxis = [np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])]
        self.frame_counter = {'count': 0}
        self.accelerations_history = []
        self.gravity_vector = [0, 0, -9.88]
        self.position_history = []
        self.Start_IMU()

        
        
    def Start_IMU(self):
        self.time ,self.gyro_x , self.gyro_y , self.gyro_z , self.accel_x , self.accel_y , self.accel_z = read_sensor_data(self.frame_counter['count'])
        self.current_time = self.time
        if not self.position_history:
            self.last_time = self.time
        self.delta_t = self.current_time - self.last_time
        
        self.gyro_data = [self.gyro_x , self.gyro_y , self.gyro_z]
        self.accel_data = [self.accel_x , self.accel_y , self.accel_z]
        
        gyro_quat = gyro_to_quaternion(self.gyro_data, self.delta_t)
        self.current_orientation = self.current_orientation * gyro_quat
        self.xyzaxis = rotate_vector_by_quaternion(self.xyzaxis, gyro_quat)
        self.gravity_vector = rotate_vector_by_quaternion([0, 0, -9.81], self.current_orientation.conjugate()) # check if conjugate is needed
        self.linear_accel = rotate_vector_by_quaternion(self.accel_data, self.current_orientation.conjugate()) - self.gravity_vector
        self.accelerations_history.append(self.linear_accel)
        
        if len(self.accelerations_history) > 3:
            self.accelerations_history.pop(0)
            
        self.velocity, self.position = self.integrate_acceleration()
        self.position_history.append(self.position)
        self.last_time = self.current_time
        self.frame_counter['count'] += 1

        Data={  
        "time " : self.time ,
        "frame_counter " : self.frame_counter ,
        "current_orientation " : self.current_orientation.to_dict() ,
        "xyzaxis " : self.xyzaxis.tolist() ,
        "linear_accel " : self.linear_accel.tolist() ,
        "velocity " : self.velocity.tolist() ,
        "position " : self.position.tolist() ,
        "delta_t " : self.delta_t 
        }
        self.Save_log(Data)
        return self.current_orientation , self.xyzaxis , self.linear_accel , self.velocity , self.position , self.delta_t , self.position_history , self.accelerations_history 



    def integrate_acceleration(self):
        """
        Intégration de l'accélération pour mettre à jour la vitesse et la position.
        Utilise une méthode plus simple au début et passe à la méthode de Simpson lorsque suffisamment de données sont disponibles.
        """
        # logging.info("Integrating acceleration...")
        if len(self.accelerations_history) < 3:
            # Si moins de 3 points sont disponibles, utilisez une méthode plus simple
            # Par exemple, méthode du point milieu ou du trapèze
            new_velocity = self.velocity + self.accelerations_history[-1] * self.delta_t
            new_position = self.position + new_velocity * self.delta_t
        else:
            # Utilisation de la méthode de Simpson
            a_n_minus_1, a_n, a_n_plus_1 = self.accelerations_history[-3:]
            new_velocity = self.velocity + (self.delta_t / 3) * (a_n_minus_1 + 4 * a_n + a_n_plus_1)
            new_position = self.position + new_velocity * self.delta_t  # Ici, une méthode simple peut toujours être utilisée

        return new_velocity, new_position
    
    @staticmethod
    def Save_log(data):
        file_path = "./log/log-{}.json".format(time.strftime("%Y-%m-%d-%H"))
        try:
            with open(file_path, 'r') as file:
                file_data = json.load(file)
                file_data.append(data)
        except FileNotFoundError:
            file_data = [data]

        with open(file_path, 'w') as file:
            json.dump(file_data, file, indent=4)


class IMUVisualizer:
    def __init__(self, processor=SensorDataProcessor()):
        self.processor = processor
        self.fig, self.ax, self.line = self.init_plot()
        self.quiver_objects = []
        self.frame_counter = {'count': 0}
        self.Reference_quiver_x = None


    def init_plot(self):
        logging.info("Initializing the plot...")
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        line, = ax.plot([], [], [], 'r-')  # 'r-' is the color and line style
        line.set_data([], [])   
        line.set_3d_properties([]) 
        ax.set_aspect('equal')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        return fig, ax, line

    def update_plot(self):
        
        self.processor.Start_IMU()
        
        # Update the line data for the plot
        xs, ys, zs = zip(*self.processor.position_history)
        self.line.set_data(np.array([xs, ys]))
        self.line.set_3d_properties(np.array(zs))
        
        padding = 1.0  # You can adjust the padding as needed
        self.ax.set_xlim([min(xs) - padding, max(xs) + padding])
        self.ax.set_ylim([min(ys) - padding, max(ys) + padding])
        self.ax.set_zlim([min(zs) - padding, max(zs) + padding])
        self.ax.set_aspect('equal')
        
        
        x_axis , y_axis , z_axis = self.processor.xyzaxis

        x_limits = self.ax.get_xlim3d()
        range = abs(x_limits[1] - x_limits[0])
        axis_length = range/5 
        

        
        # Update axes for the latest position
        if self.quiver_objects:
            try:
                Reference_quiver_x.remove()
                Reference_quiver_y.remove()
                Reference_quiver_z.remove()
            except:
                pass
            for quiv in self.quiver_objects:
                quiv.remove()

        
        Reference_quiver_x = self.ax.quiver(0,0,0,1,0,0, color='r', length=axis_length, normalize=True)
        Reference_quiver_y = self.ax.quiver(0,0,0,0,1,0, color='g', length=axis_length, normalize=True)
        Reference_quiver_z = self.ax.quiver(0,0,0,0,0,1, color='b', length=axis_length, normalize=True)
        quiver_x = self.ax.quiver(*self.processor.position, *x_axis, color='r', length=axis_length, normalize=True)
        quiver_y = self.ax.quiver(*self.processor.position, *y_axis, color='g', length=axis_length, normalize=True)
        quiver_z = self.ax.quiver(*self.processor.position, *z_axis, color='b', length=axis_length, normalize=True)

        
        self.quiver_objects[:] = [quiver_x, quiver_y, quiver_z]
        self.processor.xyzaxis[:] =[x_axis,y_axis,z_axis]
        
        return self.line, self.quiver_objects



    def animate(self,frame):
        if self.frame_counter['count'] >= 100:
            self.ani.event_source.stop()
            logging.info("Stopping the animation...")
            return self.line,

        updated_line, self.quiver_objects = self.update_plot()
        
        self.frame_counter['count'] += 1
        return updated_line

    def start_animation(self):
        self.ani = FuncAnimation(self.fig, self.animate, frames=np.arange(1000), interval=100, blit=False)
        plt.show()
        






       

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