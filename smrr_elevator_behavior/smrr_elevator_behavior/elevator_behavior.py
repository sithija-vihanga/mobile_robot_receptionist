from time import sleep
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees import logging as log_tree

import threading
from rclpy.parameter import Parameter

# Button Detection
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import rclpy.parameter
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
import yaml

from .include.button_detection_utils import ButtonDetectionUtils
import numpy as np

# Line Estimation
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from .include.line_estimation_utils import LineEstimationUtils

# Button Localization
from .include.button_localization_utils import ButtonLocalizationUtils
from smrr_interfaces.srv import Pose
from smrr_interfaces.srv import ArmControl

bridge = CvBridge()

is_elevator_bt_active = True

class WaitCMD(Behaviour, Node):
  def __init__(self, name):
    Behaviour.__init__(self, name)
    Node.__init__(self, name)

  def setup(self):
    self.logger.debug(f"WaitCMD::setup {self.name}")

    self.start_elevator_bt = self.create_service(ArmControl, 'start_elevator_bt', self.start_bt)
    
  def initialise(self):
    self.start_cmd = False
    
    self.logger.debug(f"WaitCMD::initialise {self.name}")

  def update(self):
    rclpy.spin_once(self)
    self.logger.debug(f"Waiting for elevator cmds...")
    if(self.start_cmd):
        return Status.SUCCESS
    return Status.RUNNING


  def terminate(self, new_status):
    self.logger.debug(f"WaitCMD::terminate {self.name} to {new_status}")
  
  def start_bt(self, request, response):
        if (request.start):
           self.start_cmd = True
           response.accepted = True
        return response

class ButtonDetection(Behaviour, Node):
  def __init__(self, name):
    Behaviour.__init__(self, name)
    Node.__init__(self, name)

  def setup(self):
    self.logger.debug(f"ButtonEstimation::setup {self.name}")

    self.yaml_path      = "/home/sithija/mobile_receptionist_ws/src/smrr_elevator_behavior/config/elevator_interaction.yaml"
    yolo_model_path     = "/home/sithija/mobile_receptionist_ws/src/smrr_elevator_behavior/smrr_elevator_behavior/yolo_button_detection.pt"
    self.model          = YOLO(yolo_model_path)
    
  def initialise(self):
    self.button_detection_complete = False
    self.img_sub_       = self.create_subscription(Image, "/zed2_left_camera/image_raw",self.visual_callback, 10)

    with open(self.yaml_path, 'r') as yaml_file:
            data = yaml.safe_load(yaml_file)

    self.target_button   = data['elevator_interaction']['target_button']
    self.get_logger().info("Target button is set to : %s" % self.target_button)
    
    self.button_detection_utils = ButtonDetectionUtils(self, self.target_button)

    self.x_pixel_buffer = np.array([])
    self.y_pixel_buffer = np.array([])
    self.buffer_size    = 5

    self.logger.debug(f"ButtonEstimation::initialise {self.name}")

  def update(self):
    rclpy.spin_once(self)
    self.logger.debug(f"ButtonEstimation::update {self.name}")
    if(self.button_detection_complete):
      self.logger.debug(f"ButtonEstimation::Complete {self.name}")
      return Status.SUCCESS
    return Status.RUNNING

  def terminate(self, new_status):
    self.logger.debug(f"ButtonEstimation::terminate {self.name} to {new_status}")
  
  def visual_callback(self, msg):
    self.get_logger().info("Camera callback running")
    if (len(self.x_pixel_buffer) < self.buffer_size) :
        self.get_logger().info("Collecting samples")
        pixel_x, pixel_y = self.button_detection_utils.camera_callback(
            msg, self.model, self.target_button)
        
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

    self.yaml_path        = "/home/sithija/mobile_receptionist_ws/src/smrr_elevator_behavior/config/elevator_interaction.yaml"

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
    rclpy.spin_once(self)
    self.logger.debug(f"LineEstimation::update {self.name}")
    if(self.line_estimation_complete):
      return Status.SUCCESS
    return Status.RUNNING

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

            depth_mean  = float(np.mean(self.depth_buffer) - 0.3)
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

    self.yaml_path    = "/home/sithija/mobile_receptionist_ws/src/smrr_elevator_behavior/config/elevator_interaction.yaml"

    self.get_pose_client    = self.create_client(Pose, 'get_arm_angles')
    self.arm_motion_client  = self.create_client(ArmControl, 'start_arm_motion')

  def initialise(self):
    self.logger.debug(f"ButtonLocalization::initialise {self.name}")
    self.data         = self.button_localization_utils.read_yaml(self.yaml_path)

  def update(self):
    global is_elevator_bt_active
    self.logger.debug(f"ButtonLocalization::update {self.name}")
    self.estimate_pose()
    is_elevator_bt_active = False

    self.pose_request = Pose.Request()
    self.pose_request.get_pose = True
    pose_future = self.get_pose_client.call_async(self.pose_request)
    self.logger.debug(f"Pose to angle calc request sent")

    if(pose_future):
      self.motion_request = ArmControl.Request()
      self.motion_request.start = True
      motion_future = self.arm_motion_client.call_async(self.motion_request)
      self.logger.debug(f"Arm motion request sent")
      return Status.SUCCESS
    
    else:
      return Status.RUNNING

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

  wait_cmd              = WaitCMD("wait_cmd")
  button_detection      = ButtonDetection("button_detection")
  line_estimation       = LineEstimation("estimate_line")
  button_localization   = ButtonLocalization("button_localization")

  root.add_children(
      [
          wait_cmd,
          button_detection,
          line_estimation,
          button_localization
      ]
  )
  for child in root.children:
    child.setup()  

  return root

def main(args=None):
    rclpy.init(args=args)

    log_tree.level = log_tree.Level.DEBUG
    tree = make_bt()

    try:
        while rclpy.ok():
            tree.tick_once()  
            sleep(0.1)        
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()