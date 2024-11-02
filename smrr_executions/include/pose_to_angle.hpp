#ifndef POSE_TO_ANGLE_HPP
#define POSE_TO_ANGLE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "smrr_interfaces/srv/pose.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <chrono>

class PosetoAngle : public rclcpp::Node
{
    public:
        PosetoAngle();

    private:
        moveit::planning_interface::MoveGroupInterface move_group_interface;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string yaml_path_ ;

        rclcpp::Service<smrr_interfaces::srv::Pose>::SharedPtr service;

        void timerCallback(const std::shared_ptr<smrr_interfaces::srv::Pose::Request> request,
          std::shared_ptr<smrr_interfaces::srv::Pose::Response> response);

        bool calculate_target_angles(std::string pose, 
                               std::string joint_angles_name);

        std::pair<bool, std::vector<double>> pose_to_angle(
            geometry_msgs::msg::Pose target_pose, 
            moveit::planning_interface::MoveGroupInterface& move_group_interface);
  
};

#endif