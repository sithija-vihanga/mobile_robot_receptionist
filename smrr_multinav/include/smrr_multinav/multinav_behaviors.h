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

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

#include <chrono>
#include <functional>
#include <memory>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <thread>
#include <Eigen/Dense>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <fstream>

using std::placeholders::_1;
using namespace std::chrono_literals;

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
    geometry_msgs::msg::TransformStamped last_pose_;
    double roll, pitch, yaw;
    bool start_from_dock;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    geometry_msgs::msg::TransformStamped getMapToBaseLink();
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

class MultiFloorGoal : public BT::StatefulActionNode
{
public:
    MultiFloorGoal(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

    bool goal_recieved_flag_;
    int current_floor_;
    int desired_floor_;
    YAML::Node multinav;
    std::string multinav_config;
    bool start_from_dock;

private:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    
    void goal_callback(const geometry_msgs::msg::Twist & msg);
};


class ElevatorLoading : public BT::StatefulActionNode
{
public:
    ElevatorLoading(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_ptr_;
    float K_P;
    float K_D;
    bool complete_flag_;
    float current_angle;
    float prev_angle;
    float omega;
    float laser_mean;
    Eigen::MatrixXf A; 
    Eigen::VectorXf B; 
    Eigen::VectorXf X; 
    std::vector<float> filtered_points;
    BT::Optional<std::string> action_type;
    sensor_msgs::msg::LaserScan laser_scan;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr orientation_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    
    void laser_callback(const sensor_msgs::msg::LaserScan & msg);
    void scan_extractor();
};
