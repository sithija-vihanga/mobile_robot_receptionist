import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
import yaml

class LineEstimationUtils():
    def __init__(self, node, button_info_pub_):
        self.node             = node
        self.button_info_pub_ = button_info_pub_

    def point_extractor(self, lidar_data_msg, start_angle, end_angle):

        lidar_ranges          = np.array(lidar_data_msg.ranges)
        lidar_angle_min       = lidar_data_msg.angle_min
        lidar_angle_increment = lidar_data_msg.angle_increment

        lidar_angles          = np.arange(lidar_angle_min, lidar_angle_min + len(lidar_ranges) * lidar_angle_increment, lidar_angle_increment)
        lidar_x               = lidar_ranges * np.cos(lidar_angles)
        lidar_y               = lidar_ranges * np.sin(lidar_angles)

        lidar_range_ids       = np.where((lidar_angles >= start_angle) & (lidar_angles <= end_angle))[0]
        lidar_filtered_x      = lidar_x[lidar_range_ids]
        lidar_filtered_y      = lidar_y[lidar_range_ids]

        return lidar_filtered_x, lidar_filtered_y
    

    def remove_outliers(self, lidar_x, lidar_y):
        """Function to remove outliers from LIDAR points."""

        x_median = np.median(lidar_x)

        # keep the points that are within 0.05m of the median
        indices = (lidar_x > x_median - 0.05) & (lidar_x < x_median + 0.05)
        lidar_x = lidar_x[indices]
        lidar_y = lidar_y[indices]
        
        return lidar_x, lidar_y
    
    
    def least_squares_fit(self, lidar_x, lidar_y):
        """Function to fit a line to LIDAR points using least squares method."""

        A = np.vstack([lidar_x, np.ones(len(lidar_x))]).T
        m, c = np.linalg.lstsq(A, lidar_y, rcond=None)[0]

        return m, c
    

    def calculate_depth_and_grad(self, lidar_x, lidar_y, offset):
        valid_lidar_x, valid_lidar_y = self.remove_outliers(lidar_x, lidar_y)

        # Fit a line to the LIDAR points using least squares method
        m, c = self.least_squares_fit(valid_lidar_y, valid_lidar_x)
        y = np.linspace(-2, 2, 100)
        x = m * y + c

        rounded_c        = round(c, 2)                        # depth to the button from the lidar link
        rounded_c_offset = c + offset                  # depth to the button from the camera link
        rounded_c_offset = round(rounded_c_offset, 2)
        rounded_m        = round(m, 2)

        return rounded_c_offset, rounded_m
    
    
    def plot(self, lidar_x, lidar_y, offset):

        plt.clf()  # Clear the previous plot
        plt.scatter(lidar_y, lidar_x, c='red', label='Filtered LIDAR Points')

        valid_lidar_x, valid_lidar_y = self.remove_outliers(lidar_x, lidar_y)
        plt.scatter(valid_lidar_y, valid_lidar_x, c='green', label='Valid LIDAR Points')     # indipendent variable is y, dependent variable is x

        # Fit a line to the LIDAR points using least squares method
        m, c = self.least_squares_fit(valid_lidar_y, valid_lidar_x)
        y = np.linspace(-2, 2, 100)
        x = m * y + c
        plt.plot(y, x, c='blue', label='Least Squares Line Fit')

        plt.xlim(2, -2)  
        plt.ylim(0, 2)  
        plt.xlabel('Y (m)')
        plt.ylabel('X (m)')
        plt.title('Line Estimation Visualization')
        plt.legend()
        plt.draw()
        plt.pause(0.01)  # Short pause for plot update


    def read_yaml(self, file_path):
        with open(file_path, 'r') as yaml_file:
            data = yaml.safe_load(yaml_file)
        
        return data
    

    def update_yaml(self, file_path, new_data):
        with open(file_path, 'w') as yaml_file:
            yaml.dump(new_data, yaml_file)

