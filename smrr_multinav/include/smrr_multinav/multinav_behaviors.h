#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "yaml-cpp/yaml.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <string>


class GoToPose : public BT::StatefulActionNode
{
    public:
        GoToPose(const std::string &name,
                 const BT::NodeConfiguration &config,
                 rclcpp::Node::SharedPtr node_ptr);

        using NavigateToPose    = nav2_msgs::action::NavigateToPose;
        using GoalHandleNav     = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        rclcpp::Node::SharedPtr node_ptr_;
        rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr_;
        bool done_flag_;

        static BT::PortsList providedPorts();
        
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override{};

        void nav_to_pose_callback(const GoalHandleNav::WrappedResult &result);

};