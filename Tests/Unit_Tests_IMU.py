import sys
import os

# Get the directory of the current file
current_dir = os.path.dirname(os.path.abspath(__file__))

# Append '../' to get the parent directory
parent_dir = os.path.join(current_dir, '../')

# Add the parent directory to sys.path
sys.path.insert(0, parent_dir)

import unittest
import IMU_Data as imu
import numpy as np

class TestIMUData(unittest.TestCase):
    
    def test_gyro_to_quaternion(self):
        gyro_data = np.array([1, 2, 3])
        delta_t = 0.1
        quat = imu.gyro_to_quaternion(gyro_data, delta_t)
        self.assertAlmostEqual(quat.w, 0.99875, places=5)
        self.assertAlmostEqual(quat.x, 0.00499, places=5)
        self.assertAlmostEqual(quat.y, 0.00998, places=5)
        self.assertAlmostEqual(quat.z, 0.01497, places=5)
        
    def test_rotate_vector_by_quaternion(self):
        vector = np.array([1, 0, 0])
        quat = imu.Quaternion(0.7071, 0.0, 0.0, 0.7071)
        rotated_vector = imu.rotate_vector_by_quaternion(vector, quat)
        self.assertAlmostEqual(rotated_vector[0], 0.0, places=5)
        self.assertAlmostEqual(rotated_vector[1], 1.0, places=5)
        self.assertAlmostEqual(rotated_vector[2], 0.0, places=5)
        
    def test_integrate_acceleration(self):
        acceleration = np.array([0, 0, 9.81])
        velocity = np.array([0, 0, 0])
        position = np.array([0, 0, 0])
        delta_t = 0.1
        new_velocity, new_position = imu.integrate_acceleration(acceleration, velocity, position, delta_t,new=False)
        self.assertAlmostEqual(new_velocity[0], 0.0, places=5)
        self.assertAlmostEqual(new_velocity[1], 0.0, places=5)
        self.assertAlmostEqual(new_velocity[2], 0.981, places=5)
        self.assertAlmostEqual(new_position[0], 0.0, places=5)
        self.assertAlmostEqual(new_position[1], 0.0, places=5)
        self.assertAlmostEqual(new_position[2], 0.04905, places=5)
        
    def test_integrate_velocity(self):
        acceleration = np.array([0, 0, 9.81])
        velocity = np.array([0, 0, 0])
        delta_t = 0.1
        new_velocity = imu.integrate_velocity(acceleration, velocity, delta_t)
        self.assertAlmostEqual(new_velocity[0], 0.0, places=5)
        self.assertAlmostEqual(new_velocity[1], 0.0, places=5)
        self.assertAlmostEqual(new_velocity[2], 0.981, places=5)
        
    def test_integrate_position(self):
        velocity = np.array([0, 0, 9.81])
        position = np.array([0, 0, 0])
        delta_t = 0.1
        new_position = imu.integrate_position(velocity, position, delta_t)
        self.assertAlmostEqual(new_position[0], 0.0, places=5)
        self.assertAlmostEqual(new_position[1], 0.0, places=5)
        self.assertAlmostEqual(new_position[2], 0.981, places=5)
        
if __name__ == '__main__':
    unittest.main()