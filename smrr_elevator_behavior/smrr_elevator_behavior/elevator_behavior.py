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

  def initialise(self):
    self.logger.debug(f"LineEstimation::initialise {self.name}")

  def update(self):
    self.logger.debug(f"LineEstimation::update {self.name}")
    sleep(1)
    return Status.SUCCESS

  def terminate(self, new_status):
    self.logger.debug(f"LineEstimation::terminate {self.name} to {new_status}")




class ButtonLocalization(Behaviour, Node):
  def __init__(self, name):
    Behaviour.__init__(self, name)
    Node.__init__(self, name)

  def setup(self):
    self.logger.debug(f"ButtonLocalization::setup {self.name}")

  def initialise(self):
    self.logger.debug(f"ButtonLocalization::initialise {self.name}")

  def update(self):
    self.logger.debug(f"ButtonLocalization::update {self.name}")
    sleep(1)
    return Status.SUCCESS

  def terminate(self, new_status):
    self.logger.debug(f"ButtonLocalization::terminate {self.name} to {new_status}")


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