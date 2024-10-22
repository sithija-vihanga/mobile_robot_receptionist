from time import sleep
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees import logging as log_tree

# Button Detection
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import rclpy.parameter
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from .include.button_detection_utils import ButtonDetectionUtils
import numpy as np

# Line Estimation
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from .include.line_estimation_utils import LineEstimationUtils

# Button Localization
from .include.button_localization_utils import ButtonLocalizationUtils

bridge = CvBridge()

class ButtonDetection(Behaviour, Node):
  def __init__(self, name):
    Behaviour.__init__(self, name)
    Node.__init__(self, name)

  def setup(self):
    self.logger.debug(f"ButtonEstimation::setup {self.name}")

    self.yaml_path      = "/home/sadeep/mobile_receptionist_ws/src/button_localization/config/elevator_interaction.yaml"
    yolo_model_path     = "/home/sadeep/mobile_receptionist_ws/src/button_localization/button_localization/yolo_button_detection.pt"
    self.model          = YOLO(yolo_model_path)

    self.declare_parameter("target_button", "up")
    self.img_sub_       = self.create_subscription(Image, "/zed2_left_camera/image_raw",self.camera_callback, 10)

  def initialise(self):
    self.button_detection_complete = False

    self.target_button   = self.get_parameter("target_button").get_parameter_value().string_value
    self.subcriber_topic = self.get_parameter("subscriber_topic").get_parameter_value().string_value

    self.get_logger().info("Target button is set to : %s" % self.target_button)
    
    self.button_detection_utils = ButtonDetectionUtils(self, self.target_button)

    self.x_pixel_buffer = np.array([])
    self.y_pixel_buffer = np.array([])
    self.buffer_size    = 5

    self.logger.debug(f"ButtonEstimation::initialise {self.name}")

  def update(self):
    self.logger.debug(f"ButtonEstimation::update {self.name}")
    if(self.button_detection_complete):
      return Status.SUCCESS

  def terminate(self, new_status):
    self.logger.debug(f"ButtonEstimation::terminate {self.name} to {new_status}")
  
  def camera_callback(self, msg):
    if (len(self.x_pixel_buffer) < self.buffer_size) :

        pixel_x, pixel_y = self.button_detection_utils.camera_callback(
            msg, self.model, self.target_button, self.img_pub_, self.pixel_pub_)
        
        self.x_pixel_buffer = np.append(self.x_pixel_buffer, pixel_x)
        self.y_pixel_buffer = np.append(self.y_pixel_buffer, pixel_y)
    
    else:
        data = self.button_detection_utils.read_yaml(self.yaml_path)

        x_pixel_median = int(np.median(self.x_pixel_buffer))
        y_pixel_median = int(np.median(self.y_pixel_buffer))

        data['elevator_interaction']['pixel_coordinates']['x'] = x_pixel_median
        data['elevator_interaction']['pixel_coordinates']['y'] = y_pixel_median

        self.button_detection_utils.update_yaml(self.yaml_path, data)
        self.get_logger().info("Pixel coordinates are successfully updated.")
        self.get_logger().info("Starting the line estimation process...")

        self.button_detection_complete = True

class LineEstimation(Behaviour, Node):
  def __init__(self, name):
    Behaviour.__init__(self, name)
    Node.__init__(self, name)

  def setup(self):
    self.logger.debug(f"LineEstimation::setup {self.name}")
    self.declare_parameter("start_angle", -45)
    self.declare_parameter("end_angle", 0)

    self.yaml_path        = "/home/sadeep/mobile_receptionist_ws/src/button_localization/config/elevator_interaction.yaml"

    self.laser_sub_       = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
    self.button_info_pub_ = self.create_publisher(Float32MultiArray, '/button_localization/button_info', 1)
    
    self.line_estimation_utils = LineEstimationUtils(self,self.button_info_pub_)

  def initialise(self):
    self.logger.debug(f"LineEstimation::initialise {self.name}")
    
    self.line_estimation_complete = False

    self.start_angle      = self.get_parameter("start_angle").value
    self.end_angle        = self.get_parameter("end_angle").value

    self.offset           = 0.17 # Offset in meters (offset from lidar link to left camera link optical)

    self.gradient_buffer  = np.array([])
    self.depth_buffer     = np.array([])
    self.buffer_size      = 5


  def update(self):
    self.logger.debug(f"LineEstimation::update {self.name}")
    if(self.line_estimation_complete):
      return Status.SUCCESS

  def terminate(self, new_status):
    self.logger.debug(f"LineEstimation::terminate {self.name} to {new_status}")

  def lidar_callback(self, lidar_data_msg):

        lidar_filtered_x, lidar_filtered_y = self.line_estimation_utils.point_extractor(lidar_data_msg, self.start_angle, self.end_angle)
        self.line_estimation_utils.plot(lidar_filtered_x, lidar_filtered_y, self.offset)
      
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
            
            self.line_estimation_complete = True




class ButtonLocalization(Behaviour, Node):
  def __init__(self, name):
    Behaviour.__init__(self, name)
    Node.__init__(self, name)

  def setup(self):
    self.logger.debug(f"ButtonLocalization::setup {self.name}")
    
    self.button_localization_utils = ButtonLocalizationUtils(self)

    self.yaml_path    = "/home/sadeep/mobile_receptionist_ws/src/button_localization/config/elevator_interaction.yaml"

  def initialise(self):
    self.logger.debug(f"ButtonLocalization::initialise {self.name}")
    self.data         = self.button_localization_utils.read_yaml(self.yaml_path)

  def update(self):
    self.logger.debug(f"ButtonLocalization::update {self.name}")
    self.estimate_pose()
    return Status.SUCCESS

  def terminate(self, new_status):
    self.logger.debug(f"ButtonLocalization::terminate {self.name} to {new_status}")
  
  def estimate_pose(self):

        grad_   = self.data['elevator_interaction']['gradient']
        depth_  = self.data['elevator_interaction']['depth']
        pixel_x = self.data['elevator_interaction']['pixel_coordinates']['x'] 
        pixel_y = self.data['elevator_interaction']['pixel_coordinates']['y'] 

        normal  = self.button_localization_utils.normal_vector_calculation(grad_)
        initial_pose, target_pose = self.button_localization_utils.pose_calculation(pixel_x, pixel_y, normal,depth_)

        self.data['elevator_interaction']['initial_pose'] = {
            'x':     round(float(initial_pose.position.x), 3),
            'y':     round(float(initial_pose.position.y), 3),
            'z':     round(float(initial_pose.position.z), 3),
            'roll':  0.0,  
            'pitch': 0.0,  
            'yaw':   0.0,   
        }

        self.data['elevator_interaction']['target_pose'] = {
            'x':     round(float(target_pose.position.x), 3),
            'y':     round(float(target_pose.position.y), 3),
            'z':     round(float(target_pose.position.z), 3),
            'roll':  0.0,
            'pitch': 0.0,
            'yaw':   0.0,
        }

        self.button_localization_utils.update_yaml(self.yaml_path, self.data)
        self.get_logger().info("Poses are successfully updated.")


def make_bt():
  root = Sequence(name="elevator_interaction", memory=True)

  button_detection      = ButtonDetection("button_detection")
  line_estimation       = LineEstimation("estimate_line")
  button_localization   = ButtonLocalization("button_localization")

  root.add_children(
      [
          button_detection,
          line_estimation,
          button_localization
      ]
  )

  return root


if __name__ == "__main__":
  log_tree.level = log_tree.Level.DEBUG
  tree = make_bt()
  tree.tick_once()