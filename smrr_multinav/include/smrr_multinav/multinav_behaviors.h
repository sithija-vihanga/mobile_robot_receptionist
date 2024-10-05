#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "yaml-cpp/yaml.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <string>

#include "slam_toolbox/srv/deserialize_pose_graph.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"
#include <future>

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

class LoadMapFromSlam : public BT::StatefulActionNode
{
public:
    LoadMapFromSlam(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Client<slam_toolbox::srv::DeserializePoseGraph>::SharedPtr client_;
    std::shared_future<slam_toolbox::srv::DeserializePoseGraph::Response::SharedPtr> future_;
    bool map_loading_done_flag_;
};

class WaitEvent : public BT::StatefulActionNode
{
public:
    WaitEvent(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

    bool wait_event_flag_;

private:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    
    void wait_event_callback(const std_msgs::msg::Bool & msg);
};
