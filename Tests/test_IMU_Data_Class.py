import sys
import os

# Get the directory of the current file
current_dir = os.path.dirname(os.path.abspath(__file__))

# Append '../' to get the parent directory
parent_dir = os.path.join(current_dir, '../')

# Add the parent directory to sys.path
sys.path.insert(0, parent_dir)
import numpy as np
import unittest
from IMU_Data_Class import IMUVisualizer

class TestIMUVisualizer(unittest.TestCase):
    def test_init(self):
        imu = IMUVisualizer()
        self.assertIsNotNone(imu.processor)
        self.assertIsNotNone(imu.fig)
        self.assertIsNotNone(imu.ax)
        self.assertIsNotNone(imu.line)
        self.assertEqual(imu.positions_list, [])
        self.assertEqual(imu.quiver_objects, [])
        self.assertEqual(imu.accelerations_history, [])
        self.assertEqual(imu.xyzaxis, [np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])])
        self.assertEqual(imu.frame_counter, {'count': 0})

    def test_init_plot(self):
        imu = IMUVisualizer()
        fig, ax, line = imu.init_plot()
        self.assertIsNotNone(fig)
        self.assertIsNotNone(ax)
        self.assertIsNotNone(line)

    def test_update_plot(self):
        imu = IMUVisualizer()
        imu.update_plot(0)
        self.assertEqual(len(imu.positions_list), 1)
        self.assertEqual(len(imu.quiver_objects), 3)
        self.assertEqual(len(imu.accelerations_history), 1)
        self.assertEqual(len(imu.xyzaxis), 3)

    def test_animate(self):
        imu = IMUVisualizer()
        imu.start_animation()
        self.assertEqual(imu.frame_counter['count'], 160)

if __name__ == '__main__':
    unittest.main()