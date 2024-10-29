#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>

using std::placeholders::_1;

class MovePoseNode : public rclcpp::Node
{
public:
  MovePoseNode() : Node("move_pose_node"),
                   move_group_interface(std::make_shared<rclcpp::Node>("moveit_node"), "arm")
  {
    sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
        "pose_topic", 5, std::bind(&MovePoseNode::msgCallback, this, _1));

    move_group_interface.setPlanningTime(5.0);
    move_group_interface.setPoseReferenceFrame("base_link");
    move_group_interface.setGoalPositionTolerance(0.001);
    move_group_interface.setGoalOrientationTolerance(3.14159);

    target_poses.poses.resize(3);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  moveit::planning_interface::MoveGroupInterface move_group_interface;
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::PoseArray target_poses;
  float threshold = 0.01;
  
  void msgCallback(const geometry_msgs::msg::PoseArray &msg) 
  {   
    if ( 
         (std::abs(msg.poses[0].position.x - target_poses.poses[0].position.x)) < threshold && 
         (std::abs(msg.poses[0].position.y - target_poses.poses[0].position.y)) < threshold && 
         (std::abs(msg.poses[0].position.z - target_poses.poses[0].position.z)) < threshold &&

         (std::abs(msg.poses[1].position.x - target_poses.poses[1].position.x)) < threshold &&
         (std::abs(msg.poses[1].position.y - target_poses.poses[1].position.y) )< threshold &&
         (std::abs(msg.poses[1].position.z - target_poses.poses[1].position.z)) < threshold
        )
        
    {   
        RCLCPP_INFO_STREAM(this->get_logger(), "Same pose received. Ignoring ... ");
        return;
    }
    
    RCLCPP_INFO_STREAM(this->get_logger(), "New pose received. Moving the arm to the target poses");
    target_poses = msg;

    for (int i = 0; i < target_poses.poses.size(); i++)
    {

        move_group_interface.setStartStateToCurrentState();
        move_group_interface.setPositionTarget(target_poses.poses[i].position.x, target_poses.poses[i].position.y, target_poses.poses[i].position.z, "eef_r_link" );

        // Create a plan to that target pose
        auto const [success, plan] = [&move_group_interface = move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
        }();

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planner SUCCEEDED, moving the arm and the gripper");
            move_group_interface.execute(plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "One or more planners failed!");
            return;
        }
    }

    std::vector<double> target_joint_values = {0.0, 0.0, 0.0, 0.0 };
    move_group_interface.setJointValueTarget(target_joint_values);
    move_group_interface.move();
  }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MovePoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  
}