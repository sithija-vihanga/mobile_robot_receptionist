import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    button_localization_dir = get_package_share_directory("button_localization")
   
    visualization_node = Node(
        package="button_localization",
        executable="visualizer_node"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(button_localization_dir, "config", "button_localization.rviz")],
    )

    return LaunchDescription([
        visualization_node,
        rviz_node
    ])