import rclpy
from rclpy.node import Node
from .include.button_localization_utils import ButtonLocalizationUtils


class ButtonLocalization(Node):
    def __init__(self):
        super().__init__("button_localization")

        self.declare_parameter("start_pose_estimation", False)

        self.button_localization_utils = ButtonLocalizationUtils(self)

        self.yaml_path    = "/home/sadeep/mobile_receptionist_ws/src/button_localization/config/elevator_interaction.yaml"
        self.time_        = self.create_timer(0.1, self.check_pose_estimation_param)
        self.data         = self.button_localization_utils.read_yaml(self.yaml_path)

        self.get_logger().info("Button Localization Node has been started")

    def check_pose_estimation_param(self):
        start_pose_estimation = self.get_parameter("start_pose_estimation").get_parameter_value().bool_value

        if (start_pose_estimation):
            self.get_logger().info("Starting button pose estimation.")
            self.estimate_pose()


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

        self.set_parameters([rclpy.parameter.Parameter("start_pose_estimation", rclpy.Parameter.Type.BOOL, False)])


def main(args=None):
    rclpy.init(args=args)

    button_localization_node = ButtonLocalization()

    try:
        rclpy.spin(button_localization_node)
    except KeyboardInterrupt:
        pass
    finally:
        button_localization_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()