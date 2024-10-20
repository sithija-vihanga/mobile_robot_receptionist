from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from .include.button_detection_utils import ButtonDetectionUtils


bridge = CvBridge()

class ButtonDetection(Node):
    def __init__(self):
        super().__init__("button_detection")

        self.declare_parameter("target_button", "up")
        self.declare_parameter("subscriber_topic", "/zed2_left_camera/image_raw")

        self.target_button  = self.get_parameter("target_button").get_parameter_value().string_value
        self.subcriber_topic = self.get_parameter("subscriber_topic").get_parameter_value().string_value

        self.get_logger().info("Target button is set to : %s" % self.target_button)
        self.get_logger().info("Subscriber topic is set to : %s" % self.subcriber_topic)

        path                = "/home/sadeep/mobile_receptionist_ws/src/button_localization/button_localization/yolo_button_detection.pt"
        self.model          = YOLO(path)

        self.img_sub_       = self.create_subscription(Image, "/zed2_left_camera/image_raw",self.camera_callback, 10)
        self.img_pub_       = self.create_publisher(Image, "/button_localization/detected_buttons", 10)
        self.pixel_pub_     = self.create_publisher(Int16MultiArray, "/button_localization/pixel_coordinates", 10)

        self.button_detection_utils = ButtonDetectionUtils(self,
                                                        self.target_button,
                                                        self.img_pub_,
                                                        self.pixel_pub_)


    def camera_callback(self, msg):
        
        self.button_detection_utils.camera_callback(
            msg, self.model, self.target_button, self.img_pub_, self.pixel_pub_
        )

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