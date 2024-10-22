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

class ButtonDetection(Node):
    def __init__(self):
        super().__init__("button_detection")
        
        self.declare_parameter("start_button_detection", False)
        self.declare_parameter("target_button", "up")
        self.declare_parameter("subscriber_topic", "/zed2_left_camera/image_raw")

        self.target_button   = self.get_parameter("target_button").get_parameter_value().string_value
        self.subcriber_topic = self.get_parameter("subscriber_topic").get_parameter_value().string_value

        self.get_logger().info("Target button is set to : %s" % self.target_button)
        self.get_logger().info("Subscriber topic is set to : %s" % self.subcriber_topic)

        self.yaml_path      = "/home/sadeep/mobile_receptionist_ws/src/button_localization/config/elevator_interaction.yaml"
        yolo_model_path     = "/home/sadeep/mobile_receptionist_ws/src/button_localization/button_localization/yolo_button_detection.pt"
        self.model          = YOLO(yolo_model_path)

        self.img_sub_       = self.create_subscription(Image, "/zed2_left_camera/image_raw",self.camera_callback, 10)

        self.button_detection_utils = ButtonDetectionUtils(self, self.target_button)
        
        self.x_pixel_buffer = np.array([])
        self.y_pixel_buffer = np.array([])
        self.buffer_size    = 5


    def camera_callback(self, msg):
        
        start_button_detection = self.get_parameter("start_button_detection").get_parameter_value().bool_value

        if (start_button_detection):
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

                self.set_parameters([rclpy.parameter.Parameter("start_button_detection", rclpy.Parameter.Type.BOOL, False)])
                self.set_parameters([rclpy.parameter.Parameter("start_line_estimation",  rclpy.Parameter.Type.BOOL, True )])

        else:
            return

   

def main(args=None):
    rclpy.init(args=args)

    button_detection_node = ButtonDetection()

    try:
        rclpy.spin(button_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        button_detection_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()