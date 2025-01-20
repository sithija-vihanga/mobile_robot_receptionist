import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("smrr_description"),
                    "urdf",
                    "smrr_description.urdf",
                ),
                " is_sim:=False"
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": False},  #"robot_description": robot_description,
            os.path.join(
                get_package_share_directory("smrr_controller"),
                "config",
                "arm_controller.yaml",
            ),
        ],
        remappings=[('/controller_manager/robot_description', '/robot_description')]  # Remap the topic
    )

    return LaunchDescription(
        [
            # robot_state_publisher_node,
            controller_manager,
        ]
    )