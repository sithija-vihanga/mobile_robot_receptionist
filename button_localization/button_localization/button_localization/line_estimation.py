import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt
from .include.line_estimation_utils import LineEstimationUtils

class LineEstimator(Node):
    def __init__(self):
        super().__init__('line_estimate_node')

        self.declare_parameter("start_line_estimation", False)
        self.declare_parameter("start_angle", -45)
        self.declare_parameter("end_angle", 0)

        self.start_angle      = self.get_parameter("start_angle").value
        self.end_angle        = self.get_parameter("end_angle").value

        self.offset           = 0.17 # Offset in meters (offset from lidar link to left camera link optical)

        self.laser_sub_       = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.button_info_pub_ = self.create_publisher(Float32MultiArray, '/button_localization/button_info', 1)

        self.gradient_buffer  = np.array([])
        self.depth_buffer     = np.array([])
        self.buffer_size      = 5

        self.yaml_path        = "/home/sadeep/mobile_receptionist_ws/src/button_localization/config/elevator_interaction.yaml"

        self.line_estimation_utils = LineEstimationUtils(self,self.button_info_pub_)

        self.get_logger().info("Lidar processor node has been started")


    def lidar_callback(self, lidar_data_msg):

        lidar_filtered_x, lidar_filtered_y = self.line_estimation_utils.point_extractor(lidar_data_msg, self.start_angle, self.end_angle)
        self.line_estimation_utils.plot(lidar_filtered_x, lidar_filtered_y, self.offset)


        start_line_estimation = self.get_parameter("start_line_estimation").get_parameter_value().bool_value
        
        if (start_line_estimation) :
            
            if (len(self.gradient_buffer) < self.buffer_size ):
                
                depth_ , grad_ = self.line_estimation_utils.calculate_depth_and_grad(lidar_filtered_x, lidar_filtered_y, self.offset)

                self.gradient_buffer = np.append(self.gradient_buffer, grad_)
                self.depth_buffer    = np.append(self.depth_buffer, depth_)

            else:
                data  = self.line_estimation_utils.read_yaml(self.yaml_path)

                depth_mean  = float(np.mean(self.depth_buffer))
                grad_median = float(np.median(self.gradient_buffer))

                data['elevator_interaction']['depth']    = depth_mean
                data['elevator_interaction']['gradient'] = grad_median

                self.line_estimation_utils.update_yaml(self.yaml_path, data)
                self.get_logger().info("Depth and Gradient values are successfully updated.")
                self.get_logger().info(" Starting the pose estimation process...")

                self.set_parameters([rclpy.parameter.Parameter("start_line_estimation", rclpy.Parameter.Type.BOOL, False)])
                self.set_parameters([rclpy.parameter.Parameter("start_pose_estimation",  rclpy.Parameter.Type.BOOL, True )])

        else:
            return

                
                
  
def main(args=None):
    rclpy.init(args=args)

    line_estimator_node = LineEstimator()

    try:
        rclpy.spin(line_estimator_node)
    except KeyboardInterrupt:
        pass
    finally:
        line_estimator_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
