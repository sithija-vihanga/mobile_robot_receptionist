#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <chrono>

#include "pose_to_angle.hpp"

using std::placeholders::_1;


PosetoAngle::PosetoAngle() : Node("pose_to_angle_node"),
                  move_group_interface(std::make_shared<rclcpp::Node>("moveit_node"), "arm")
  {

    this->declare_parameter<bool>("start_joint_calculations", false);

    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&PosetoAngle::timerCallback, this)) ;

    yaml_path_ = "/home/sadeep/mobile_receptionist_ws/src/button_localization/config/elevator_interaction.yaml" ;

    move_group_interface.setPlanningTime(5.0);
    move_group_interface.setPoseReferenceFrame("base_link");
    move_group_interface.setGoalPositionTolerance(0.001);
    move_group_interface.setGoalOrientationTolerance(3.14159);

  }
  

  void PosetoAngle::timerCallback()
  {
    bool start_joint_calculations ;
    this->get_parameter("start_joint_calculations", start_joint_calculations);

    if (start_joint_calculations)
    {
      RCLCPP_INFO(rclcpp::get_logger("pose_to_angle"), "Starting to find target joint angles.");
      bool ok1 = calculate_target_angles("initial_pose", "initial_joint_angles");
      bool ok2 = calculate_target_angles("target_pose" , "target_joint_angles" );
      bool ok3 = calculate_target_angles("initial_pose", "end_joint_angles"    );

      if (ok1 && ok2)
      {
        RCLCPP_INFO(rclcpp::get_logger("pose_to_angle"), "Target joint angles are successfully updated.");
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("pose_to_angle"), "Failed to find target joint angles.");

      }

      this->set_parameter(rclcpp::Parameter("start_joint_calculations", false));
      this->set_parameter(rclcpp::Parameter("start_arm_control", true));
      rclcpp::shutdown();
      
    }
  }

  bool PosetoAngle::calculate_target_angles(std::string pose, 
                               std::string joint_angles_name)
  {
      YAML::Node data;
    try {
        data = YAML::LoadFile(this->yaml_path_);
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pose_to_angle"), "Failed to load YAML file: %s", e.what());
        rclcpp::shutdown();
        return false;
    }

    if (!data["elevator_interaction"][pose]) {
        RCLCPP_ERROR(rclcpp::get_logger("pose_to_angle"), "Pose %s not defined in YAML.", pose.c_str());
        rclcpp::shutdown();
        return false;
    }

      double x = data["elevator_interaction"][pose]["x"].as<double>();
      double y = data["elevator_interaction"][pose]["y"].as<double>();
      double z = data["elevator_interaction"][pose]["z"].as<double>();
      RCLCPP_INFO(rclcpp::get_logger("done"), "done");

      geometry_msgs::msg::Pose target_pose ;
      target_pose.position.x  = x ;
      target_pose.position.y  = y ;
      target_pose.position.z  = z ;

      const auto [success, target_joint_angles] = pose_to_angle(target_pose, move_group_interface);

      if (success)
      {
        data["elevator_interaction"][joint_angles_name]["joint1"] = target_joint_angles[0];
        data["elevator_interaction"][joint_angles_name]["joint2"] = target_joint_angles[1];
        data["elevator_interaction"][joint_angles_name]["joint3"] = target_joint_angles[2];
        data["elevator_interaction"][joint_angles_name]["joint4"] = target_joint_angles[3];
        std::ofstream fout(this->yaml_path_);
        fout << data;

        return true;
      }
      else
      {
        return false;
      }
  }

  std::pair<bool, std::vector<double>> PosetoAngle::pose_to_angle(geometry_msgs::msg::Pose target_pose, 
                    moveit::planning_interface::MoveGroupInterface& move_group_interface)
  {

        move_group_interface.setStartStateToCurrentState();
        move_group_interface.setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z, "eef_r_link" );

        // Create a plan to that target pose
        auto const [success, plan] = [&move_group_interface = move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
        }();

        if (success)
        {
            RCLCPP_INFO(rclcpp::get_logger("pose_to_angle"), "Planner succeeded, finding the target joint angles");
            const auto final_waypoint = plan.trajectory_.joint_trajectory.points.back();
            const std::vector<double> target_joint_angles = final_waypoint.positions;
            return {true, target_joint_angles};
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("pose_to_angle"), "One or more planners failed!");
            return{false, {}};
            rclcpp::shutdown();
        }
  };


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PosetoAngle>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  
}