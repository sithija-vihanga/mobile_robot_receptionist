import numpy as np
from geometry_msgs.msg import Pose, PoseArray, Point
from visualization_msgs.msg import Marker
import yaml

class ButtonLocalizationUtils():
    def __init__(self, node ):
        self.node = node
        
        # Intrinsic camera parameters
        self.f_x   = 537.6983672158217
        self.f_y   = 537.1784833375057

        self.c_x   = 640.9944295362419  
        self.c_y   = 362.64041228998025

        # Transformation matrix from base_link to camera_link
        self.base_to_camera = np.array([
            [0.00  , 0.00  , 1.00 , 0.06],
            [-1.00 , 0.00  , 0.00 , 0.06],
            [0.00  ,-1.00  , 0.00 , 1.10],
            [0.00  , 0.00  , 0.00 , 1.00]
        ])

        # Transformation matrix from base_link to lidar_link
        self.base_to_lidar = np.array([
            [1.000, 0.000, 0.000, 0.230],
            [0.000, 1.000, 0.000, 0.000],
            [0.000, 0.000, 1.000, 0.001],
            [0.000, 0.000, 0.000, 1.000]
        ])

    def pose_calculation(self, pixel_x, pixel_y, normal, depth):
        
        X = depth * (pixel_x - self.c_x) / self.f_x
        Y = depth * (pixel_y - self.c_y) / self.f_y
        Z = depth

        point = np.array([X, Y, Z])
        transformed_point = np.dot(self.base_to_camera, np.append(point, 1))     # transform the point to the base_link frame

        offset = 0.1
        init_pose = Pose()
        init_pose.position.x        = transformed_point[0] + offset * normal[0]   # the initial point the arm is moving to
        init_pose.position.y        = transformed_point[1] + offset * normal[1]
        init_pose.position.z        = transformed_point[2] + offset * normal[2]

        offset_ = 0.0
        target_pose = Pose()
        target_pose.position.x      = transformed_point[0] + offset_ * normal[0]     # offset_ away from the goal position, if the arm hits the wall the robot will fall back in simulation.
        target_pose.position.y      = transformed_point[1] + offset_ * normal[1]
        target_pose.position.z      = transformed_point[2] + offset_ * normal[2]

        pose_array                  = PoseArray()
        pose_array.header.frame_id  = "base_link"
        pose_array.header.stamp     = self.node.get_clock().now().to_msg()
        pose_array.poses.append(init_pose)
        pose_array.poses.append(target_pose)
        pose_array.poses.append(init_pose)
        # self.pose_pub_.publish(pose_array)

        # self.pose_visualizer([init_pose.position.x, init_pose.position.y, init_pose.position.z],
        #                      [target_pose.position.x, target_pose.position.y, target_pose.position.z])

        return init_pose, target_pose
    

    def pose_visualizer(self, pose1, pose2):

        # Visualize the first pose using an RViz Marker
        marker                  = Marker()
        marker.header.frame_id  = "base_link"
        marker.header.stamp     = self.node.get_clock().now().to_msg()
        marker.ns               = "init_pose"
        marker.id               = 1
        marker.type             = Marker.SPHERE
        marker.action           = Marker.ADD
        
        marker.scale.x          = 0.05
        marker.scale.y          = 0.05
        marker.scale.z          = 0.05
        marker.color.a          = 1.0
        marker.color.r          = 0.0
        marker.color.g          = 1.0
        marker.color.b          = 0.0

        marker.pose.position.x  = pose1[0]
        marker.pose.position.y  = pose1[1]
        marker.pose.position.z  = pose1[2]
        # self.init_marker_.publish(marker)

        marker.pose.position.x  = pose2[0]
        marker.pose.position.y  = pose2[1]
        marker.pose.position.z  = pose2[2]
        # self.goal_marker_.publish(marker)

    
    def normal_vector_calculation(self, grad):

        normal = np.array([-1, grad, 0])                                          # 3D normal vector
        transformed_normal = np.dot(self.base_to_lidar, np.append(normal, 1))     # transform the normal to the lidar_link frame

        return transformed_normal
    

    def normal_visualizer(self, normal, starting_point):
        
        """Visualize the normal vector using an RViz Marker."""
        marker                  = Marker()
        marker.header.frame_id  = "base_link"  # Base frame for RViz
        marker.header.stamp     = self.node.get_clock().now().to_msg()
        marker.ns               = "normal_vector"
        marker.id               = 1
        marker.type             = Marker.ARROW  # Arrow type marker
        marker.action           = Marker.ADD

        start_point_x           = starting_point[0]
        start_point_y           = starting_point[1]
        start_point_z           = starting_point[2]

        marker.scale.x          = 0.01  # Shaft diameter
        marker.scale.y          = 0.05  # Arrowhead diameter
        marker.scale.z          = 0.2  # Arrowhead length

        marker.points           = []
        start_point             = Point()
        start_point.x           = start_point_x
        start_point.y           = start_point_y
        start_point.z           = start_point_z

        end_point               = Point()
        end_point.x             = start_point_x + (-1*normal[0])
        end_point.y             = start_point_y + (-1*normal[1])
        end_point.z             = start_point_z + (-1*normal[2])

        marker.points.append(start_point)  
        marker.points.append(end_point)

        marker.color.a          = 1.0  # Alpha (opacity)
        marker.color.r          = 1.0  # Red
        marker.color.g          = 0.0  # Green
        marker.color.b          = 0.0  # Blue

        # self.normal_pub_.publish(marker)


    def read_yaml(self, file_path):
        with open(file_path, 'r') as yaml_file:
            data = yaml.safe_load(yaml_file)
        
        return data
    

    def update_yaml(self, file_path, new_data):
        with open(file_path, 'w') as yaml_file:
            yaml.dump(new_data, yaml_file)
