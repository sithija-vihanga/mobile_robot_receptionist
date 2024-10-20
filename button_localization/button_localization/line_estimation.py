import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt
import time

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor_node')

        self.declare_parameter("start_angle", -45)
        self.declare_parameter("end_angle", 0)

        self.start_angle = self.get_parameter("start_angle").value
        self.end_angle = self.get_parameter("end_angle").value

        self.offset = 0.17 # Offset in meters (offset from lidar link to left camera link optical)

        self.laser_sub_ = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.button_info_pub_ = self.create_publisher(Float32MultiArray, '/button_localization/button_info', 1)
        self.get_logger().info("Lidar processor node has been started")

    def lidar_callback(self, lidar_data_msg):
        
        lidar_ranges          = np.array(lidar_data_msg.ranges)
        lidar_angle_min       = lidar_data_msg.angle_min
        lidar_angle_increment = lidar_data_msg.angle_increment

        lidar_angles          = np.arange(lidar_angle_min, lidar_angle_min + len(lidar_ranges) * lidar_angle_increment, lidar_angle_increment)
        lidar_x               = lidar_ranges * np.cos(lidar_angles)
        lidar_y               = lidar_ranges * np.sin(lidar_angles)

        lidar_points = np.vstack((lidar_x, lidar_y)).T

        lidar_range_ids    = np.where((lidar_angles >= self.start_angle) & (lidar_angles <= self.end_angle))[0]
        lidar_filtered_x   = lidar_x[lidar_range_ids]
        lidar_filtered_y   = lidar_y[lidar_range_ids]

        self.plot(lidar_filtered_x, lidar_filtered_y)


    def remove_outliers(self, lidar_x, lidar_y):
        """Function to remove outliers from LIDAR points."""

        # find median of x
        x_median = np.median(lidar_x)

        # keep the points that are within 0.05m of the median
        indices = (lidar_x > x_median - 0.05) & (lidar_x < x_median + 0.05)
        lidar_x = lidar_x[indices]
        lidar_y = lidar_y[indices]
        
        return lidar_x, lidar_y


    def least_squares_fit(self, lidar_x, lidar_y):
        """Function to fit a line to LIDAR points using least squares method."""

        # Fit a line to the LIDAR points using least squares method
        A = np.vstack([lidar_x, np.ones(len(lidar_x))]).T
        m, c = np.linalg.lstsq(A, lidar_y, rcond=None)[0]

        return m, c


    def plot(self, lidar_x, lidar_y):
        """Function to plot filtered LIDAR points and the least squares line fit."""
        plt.clf()  # Clear the previous plot
        plt.scatter(lidar_y, lidar_x, c='red', label='Filtered LIDAR Points')

        valid_lidar_x, valid_lidar_y = self.remove_outliers(lidar_x, lidar_y)
        plt.scatter(valid_lidar_y, valid_lidar_x, c='green', label='Valid LIDAR Points')     # indipendent variable is y, dependent variable is x

        # Fit a line to the LIDAR points using least squares method
        m, c = self.least_squares_fit(valid_lidar_y, valid_lidar_x)
        y = np.linspace(-2, 2, 100)
        x = m * y + c
        plt.plot(y, x, c='blue', label='Least Squares Line Fit')

        rounded_c = round(c, 2)                        # depth to the button from the lidar link
        rounded_c_offset = c + self.offset             # depth to the button from the camera link
        rounded_c_offset = round(rounded_c_offset, 2)

        rounded_m = round(m, 2)

        button_info = Float32MultiArray()
        button_info.data = [rounded_c_offset, rounded_m]
        self.button_info_pub_.publish(button_info)

        plt.xlim(2, -2)  # Set x-axis limits (adjust based on your environment)
        plt.ylim(0, 2)  # Set y-axis limits (adjust based on your environment)
        plt.xlabel('Y (m)')
        plt.ylabel('X (m)')
        plt.title('Line Estimation Visualization')
        plt.legend()
        plt.draw()
        plt.pause(0.01)  # Short pause for plot update

  
def main(args=None):

    rclpy.init(args=args)
    processor = LidarProcessor()
    
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
