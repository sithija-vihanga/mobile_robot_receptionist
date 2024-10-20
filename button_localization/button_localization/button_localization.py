import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Float32, Int16, Int16MultiArray
from geometry_msgs.msg import PoseArray
from cv_bridge import CvBridge
import numpy as np
from visualization_msgs.msg import Marker
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from .include.button_localization_utils import ButtonLocalizationUtils


bridge = CvBridge()
depth_, grad_    = Float32(), Float32()
pixel_x_, pixel_y_ = Int16(), Int16()


class ButtonLocalization(Node):
    def __init__(self):
        super().__init__("button_localization")

        self.cb_group     = ReentrantCallbackGroup()
        self.image_sub_   = self.create_subscription(Image, "/zed2_left_camera/image_raw", self.camera_callback, 10, callback_group=self.cb_group)
        # self.image_pub_   = self.create_publisher(Image, "/annotated_image", 1)
        self.pose_pub_    = self.create_publisher(PoseArray, "/button_localization/pose_topic", 1)                     # make it to publish pose array
        self.goal_marker_ = self.create_publisher(Marker,    "/button_localization/goal_marker", 1)
        self.init_marker_ = self.create_publisher(Marker,    "/button_localization/init_marker", 1)
        self.normal_pub_  = self.create_publisher(Marker,    "/button_localization/normal_marker", 1)

        self.button_localization_utils = ButtonLocalizationUtils(
            self, self.pose_pub_, 
            self.goal_marker_, self.init_marker_, self.normal_pub_
        )

        self.get_logger().info("Button Localization Node has been started")


    def camera_callback(self, msg):
        global depth_ , grad_, pixel_x_, pixel_y_

        # pixel representation
        # img = bridge.imgmsg_to_cv2(msg, "bgr8")
        # cv2.circle(img, (pixel_x_.data, pixel_y_.data), 15, (0, 0, 255), -1)

        # img_msg = bridge.cv2_to_imgmsg(img)
        # self.image_pub_.publish(img_msg)

        # button pose calculations
        normal = self.button_localization_utils.normal_vector_calculation(grad_)        
        init_pose, target_pose = self.button_localization_utils.pose_calculation(pixel_x_, pixel_y_,normal, depth_,)
        self.button_localization_utils.normal_visualizer(normal, [init_pose.position.x, init_pose.position.y, init_pose.position.z])


class DepthSubscriber(Node):
    def __init__(self):
        super().__init__("depth_subscriber")

        self.cb_group          = ReentrantCallbackGroup()
        self.depth_sub_        = self.create_subscription(Float32MultiArray, "/button_localization/button_info", self.depth_callback, 10, callback_group=self.cb_group)

        self.depth_arr         = np.array([])
        self.grad_arr          = np.array([])

    def depth_callback(self, msg):
        if (len(self.depth_arr) < 5):
            self.depth_arr     = np.append(self.depth_arr, msg.data[0])
            self.grad_arr      = np.append(self.grad_arr, msg.data[1])
        else:
            depth_.data        = np.mean(self.depth_arr)
            grad_.data         = np.median(self.grad_arr)
            self.depth_arr     = np.array([])
            self.grad_arr      = np.array([])


class PixelSubscriber(Node):
    def __init__(self):
        super().__init__("pixel_subscriber")

        self.cb_group          = ReentrantCallbackGroup()
        self.pixel_sub_        = self.create_subscription(Int16MultiArray, "/button_localization/pixel_coordinates", self.pixel_callback, 10, callback_group=self.cb_group)
        
        self.pixel_x_arr       = np.array([])
        self.pixel_y_arr       = np.array([])

    def pixel_callback(self, msg):
        if (len(self.pixel_x_arr) < 5):
            self.pixel_x_arr   = np.append(self.pixel_x_arr, msg.data[0])
            self.pixel_y_arr   = np.append(self.pixel_y_arr, msg.data[1])
        else:
            pixel_x_.data      = int(np.median(self.pixel_x_arr))
            pixel_y_.data      = int(np.median(self.pixel_y_arr))
            self.pixel_x_arr   = np.array([])
            self.pixel_y_arr   = np.array([])


def main(args=None):
    rclpy.init(args=args)

    button_localization = ButtonLocalization()
    depth_subscriber    = DepthSubscriber()
    pixel_subscriber    = PixelSubscriber()

    executor            = MultiThreadedExecutor()
    executor.add_node(button_localization)
    executor.add_node(depth_subscriber)
    executor.add_node(pixel_subscriber)
    executor.spin()


if __name__ == "__main__":
    main()