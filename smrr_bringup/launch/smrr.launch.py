import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("smrr_base_controller"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("smrr_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={'is_sim': 'False'}.items()
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("smrr_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
    )
    
    return LaunchDescription([
        hardware_interface,
        controller,
        joystick
    ])