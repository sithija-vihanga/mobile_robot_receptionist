from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python import get_package_share_directory

    
def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True",
    )

    is_sim = LaunchConfiguration("is_sim")

    moveit_config = (MoveItConfigsBuilder("smrr", package_name="smrr_moveit")
    .robot_description(file_path=os.path.join(get_package_share_directory("smrr_description"), "urdf", "smrr_description.urdf"))
    .robot_description_semantic(file_path="config/smrr.srdf")
    .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
    .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"]
        )
    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), 
                    {"use_sim_time": is_sim} ,
                    {"publish_robot_description_semantic": True},
                    {"publish_monitored_planning_scene": True},
                    ],
        arguments=["--ros-args", "--log-level", "info"]
    )

    rviz_config = os.path.join(get_package_share_directory("smrr_moveit"), "config", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    moveit_config.trajectory_execution,
                    moveit_config.planning_pipelines,
                    {"use_sim_time": is_sim}
        ]
    )

    return LaunchDescription([
        is_sim_arg,
        move_group_node,
        rviz_node,
    ])